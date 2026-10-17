"""
Stage 5 Ground Station Client with Camera-to-Nozzle Calibration & Benchmarking
Semi-Autonomous Indoor Fire-Extinguishing Drone Project
Target: ESP32-S3 Unified System (MJPEG Video Stream + WebSocket Telemetry)
"""

import sys
import time
import json
import math
import uuid
import threading
import argparse
import numpy as np
import cv2

try:
    import websockets
    import asyncio
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

class CalibrationEngine:
    """
    Transforms detected 2D fire coordinates (flame base) into calibrated
    3D nozzle aim angles, accounting for:
    1. Camera intrinsics (Resolution 320x240, HFOV 60°, VFOV 45°)
    2. Flame base targeting (aiming at combustion root, not flickering tip)
    3. Camera-to-nozzle physical mounting offset (lever arm vector)
    4. Ballistic water jet gravity drop delta_y(d)
    5. Operating distance (configured fixed 1.0m, pending physical ToF integration)
    """
    def __init__(self,
                 img_w=320, img_h=240,
                 hfov_deg=60.0, vfov_deg=45.0,
                 nozzle_offset_x_m=0.0,
                 nozzle_offset_y_m=-0.040, # 40mm below lens
                 nozzle_offset_z_m=0.015,  # 15mm forward of lens
                 water_speed_mps=4.0,
                 calibrated_dist_m=1.0):
        self.w = img_w
        self.h = img_h
        self.cx = img_w / 2.0
        self.cy = img_h / 2.0

        # Focal lengths in pixels
        self.fx = (img_w / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
        self.fy = (img_h / 2.0) / math.tan(math.radians(vfov_deg / 2.0))

        self.off_x = nozzle_offset_x_m
        self.off_y = nozzle_offset_y_m
        self.off_z = nozzle_offset_z_m
        self.v_water = water_speed_mps
        self.dist_m = calibrated_dist_m
        self.g = 9.81

    def compute_aim_angles(self, u_base, v_base, dist_m=None):
        if dist_m is None or dist_m <= 0:
            dist_m = self.dist_m

        # 1. Back-project to 3D point in Camera Frame
        X_c = ((u_base - self.cx) / self.fx) * dist_m
        Y_c = ((v_base - self.cy) / self.fy) * dist_m
        Z_c = dist_m

        # 2. Transform to Nozzle Frame (compensate physical mounting baseline)
        X_n = X_c - self.off_x
        Y_n = Y_c - self.off_y
        Z_n = max(0.1, Z_c - self.off_z)

        # 3. Ballistic Gravity Drop Correction
        t_flight = Z_n / self.v_water
        drop_m = 0.5 * self.g * (t_flight ** 2)

        # Aim upward to compensate for water trajectory curvature
        Y_aim = Y_n - drop_m

        # 4. Compute required Pan & Tilt angles (degrees)
        pan_deg = math.degrees(math.atan2(X_n, Z_n))
        tilt_deg = -math.degrees(math.atan2(Y_aim, Z_n))

        # 5. Convert to servo pulse widths (nominal 11.11 us/deg around 1500 us)
        # Clamped to active safe window [1333 - 1667 us]
        pan_us = max(1333.0, min(1667.0, 1500.0 + pan_deg * 11.11))
        tilt_us = max(1333.0, min(1667.0, 1500.0 + tilt_deg * 11.11))

        return {
            "pan_deg": pan_deg,
            "tilt_deg": tilt_deg,
            "pan_us": pan_us,
            "tilt_us": tilt_us,
            "drop_cm": drop_m * 100.0,
            "t_flight_ms": t_flight * 1000.0
        }

class Stage5GroundStation:
    def __init__(self, esp_ip="192.168.0.113", http_port=80, auth_token="DRONE_SECURE_AUTH_STAGE5_2026"):
        self.esp_ip = esp_ip
        self.http_port = http_port
        self.auth_token = auth_token
        self.stream_url = f"http://{esp_ip}:{http_port}/stream"
        self.ws_url = f"ws://{esp_ip}:{http_port}/ws"

        self.session_id = f"sess_{uuid.uuid4().hex[:8]}"
        self.running = True
        self.seq_num = 0
        self.lock = threading.Lock()

        self.calib = CalibrationEngine(img_w=320, img_h=240, calibrated_dist_m=1.0)

        # Telemetry received from drone
        self.telemetry = {
            "state": "DISCONNECTED",
            "fault": "NONE",
            "interlock": "INITIALIZING",
            "pan_us": 1500,
            "tilt_us": 1500,
            "bursting": False,
            "simulated": True,
            "conv_us": 0,
            "frame_seq": 0,
            "last_rx_time": 0
        }

        # Latest vision target data
        self.target_data = {
            "detected": False,
            "conf": 0.0,
            "u_base": 160.0,
            "v_base": 120.0,
            "pan_err": 0.0,
            "tilt_err": 0.0,
            "dist_m": 1.0,
            "bbox": None,
            "aim_data": None
        }

        # Latency benchmark metrics
        self.latency_stats = {
            "fps": 0.0,
            "transport_ms": 0.0,
            "detect_ms": 0.0,
            "ws_rtt_ms": 0.0,
            "total_latency_ms": 0.0
        }

        self.pending_commands = []
        self.ws_connected = False
        self.fullscreen = False

    def send_command(self, cmd_type, **kwargs):
        with self.lock:
            self.seq_num += 1
            pkt = {
                "auth": self.auth_token,
                "seq": self.seq_num,
                "ts": int(time.time() * 1000),
                "type": cmd_type
            }
            pkt.update(kwargs)
            self.pending_commands.append(pkt)

    def ws_thread_main(self):
        """Asynchronous WebSocket client loop running in background thread"""
        if not HAS_WEBSOCKETS:
            print("[WARN] websockets library not available. Telemetry disabled.")
            return

        async def ws_loop():
            while self.running:
                try:
                    async with websockets.connect(self.ws_url, ping_interval=None) as ws:
                        self.ws_connected = True
                        print(f"[WS] Connected to ESP32-S3. Establishing session handshake ({self.session_id})...")

                        # Send session handshake to reset sequence counter cleanly
                        hs_pkt = {
                            "auth": self.auth_token,
                            "type": "handshake",
                            "session_id": self.session_id
                        }
                        await ws.send(json.dumps(hs_pkt))
                        self.seq_num = 0
                        last_hb = time.time()

                        while self.running:
                            now = time.time()
                            to_send = []
                            with self.lock:
                                if self.pending_commands:
                                    to_send = list(self.pending_commands)
                                    self.pending_commands.clear()

                            for pkt in to_send:
                                t_tx = time.time()
                                await ws.send(json.dumps(pkt))

                            # Periodic heartbeat / target telemetry transmission every 100 ms
                            if now - last_hb >= 0.100:
                                self.seq_num += 1
                                is_det = self.target_data["detected"]
                                hb_pkt = {
                                    "auth": self.auth_token,
                                    "seq": self.seq_num,
                                    "ts": int(now * 1000),
                                    "type": "target" if is_det else "heartbeat"
                                }
                                if is_det:
                                    hb_pkt["detected"] = True
                                    hb_pkt["conf"] = self.target_data["conf"]
                                    hb_pkt["pan_err"] = self.target_data["pan_err"]
                                    hb_pkt["tilt_err"] = self.target_data["tilt_err"]
                                    hb_pkt["dist_m"] = self.target_data["dist_m"]
                                    hb_pkt["frame_ts"] = int(now * 1000)

                                t_tx = time.time()
                                await ws.send(json.dumps(hb_pkt))
                                last_hb = now

                            # Receive telemetry responses
                            try:
                                msg = await asyncio.wait_for(ws.recv(), timeout=0.04)
                                t_rx = time.time()
                                data = json.loads(msg)
                                with self.lock:
                                    self.telemetry["state"] = data.get("state", "UNKNOWN")
                                    self.telemetry["fault"] = data.get("fault", "NONE")
                                    self.telemetry["interlock"] = data.get("interlock", "UNKNOWN")
                                    self.telemetry["pan_us"] = data.get("pan_us", 1500)
                                    self.telemetry["tilt_us"] = data.get("tilt_us", 1500)
                                    self.telemetry["bursting"] = data.get("bursting", False)
                                    self.telemetry["simulated"] = data.get("simulated", True)
                                    self.telemetry["conv_us"] = data.get("conv_us", 0)
                                    self.telemetry["frame_seq"] = data.get("frame_seq", 0)
                                    self.telemetry["last_rx_time"] = t_rx
                                    self.latency_stats["ws_rtt_ms"] = (t_rx - last_hb) * 1000.0
                            except asyncio.TimeoutError:
                                pass

                            await asyncio.sleep(0.01)
                except Exception as e:
                    self.ws_connected = False
                    with self.lock:
                        self.telemetry["state"] = "DISCONNECTED"
                    await asyncio.sleep(1.0)

        asyncio.run(ws_loop())

    def detect_fire(self, frame):
        """
        Flame-Base Detection Engine:
        Detects combustion core and specifically identifies the lowest vertical
        pixel (flame root / fuel source) rather than the flame apex.
        """
        t0 = time.time()
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Flame color range
        lower_flame = np.array([0, 120, 180])
        upper_flame = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_flame, upper_flame)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 120:
                if area > max_area:
                    max_area = area
                    best_box = cv2.boundingRect(cnt)

        t_detect = (time.time() - t0) * 1000.0
        self.latency_stats["detect_ms"] = t_detect

        if best_box is not None:
            bx, by, bw, bh = best_box
            # CRITICAL CALIBRATION FIX: Target the FLAME BASE (bottom-center)
            u_base = float(bx + bw / 2.0)
            v_base = float(by + bh) # Bottom-most row of flame bounding box

            conf = min(0.98, max(0.40, 0.50 + (max_area / 4000.0) * 0.45))

            # Calibrated distance: 1.0 meter fixed calibration mode
            dist_m = 1.0

            # Pass coordinates into Calibration Engine
            aim = self.calib.compute_aim_angles(u_base, v_base, dist_m=dist_m)

            return True, conf, u_base, v_base, aim["pan_deg"], aim["tilt_deg"], dist_m, (bx, by, bw, bh), aim

        return False, 0.0, 160.0, 120.0, 0.0, 0.0, 1.0, None, None

    def run_gui(self):
        print(f"[STREAM] Opening video capture at {self.stream_url}...")
        cap = cv2.VideoCapture(self.stream_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ws_thread = threading.Thread(target=self.ws_thread_main, daemon=True)
        ws_thread.start()

        window_name = "Stage 5 Fire Drone Integration HUD"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        fps_timer = time.time()
        frame_count = 0
        current_fps = 0.0

        while self.running:
            t_cap_start = time.time()
            ret, frame = cap.read()
            if not ret or frame is None:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "CONNECTING TO ESP32-S3 GC2145 STREAM...", (60, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
                cv2.imshow(window_name, blank)
                key = cv2.waitKey(200) & 0xFF
                if key == ord('q') or key == 27:
                    break
                cap.release()
                time.sleep(0.5)
                cap = cv2.VideoCapture(self.stream_url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                continue

            frame_count += 1
            if time.time() - fps_timer >= 1.0:
                current_fps = frame_count / (time.time() - fps_timer)
                self.latency_stats["fps"] = current_fps
                frame_count = 0
                fps_timer = time.time()

            # Fire detection and calibration computation
            detected, conf, ub, vb, pan_err, tilt_err, dist_m, bbox, aim = self.detect_fire(frame)
            with self.lock:
                self.target_data["detected"] = detected
                self.target_data["conf"] = conf
                self.target_data["u_base"] = ub
                self.target_data["v_base"] = vb
                self.target_data["pan_err"] = pan_err
                self.target_data["tilt_err"] = tilt_err
                self.target_data["dist_m"] = dist_m
                self.target_data["bbox"] = bbox
                self.target_data["aim_data"] = aim

            # Automatically dispatch calibrated servo positions when target detected
            if detected and aim is not None and self.telemetry["state"] in ("ARMED_IDLE", "BURSTING"):
                self.send_command("servo", pan_us=aim["pan_us"], tilt_us=aim["tilt_us"])

            # Render Overlay HUD
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # 1. Optical Center Crosshairs (Gray)
            cv2.line(frame, (cx - 12, cy), (cx + 12, cy), (120, 120, 120), 1)
            cv2.line(frame, (cx, cy - 12), (cx, cy + 12), (120, 120, 120), 1)

            # 2. Alignment Tolerance Circle (3.0 deg)
            radius_px = int((3.0 / 30.0) * (w / 2.0))
            is_aligned = (abs(pan_err) <= 3.0 and abs(tilt_err) <= 3.0) if detected else False
            reticle_color = (0, 255, 0) if is_aligned else (0, 0, 255)
            cv2.circle(frame, (cx, cy), radius_px, reticle_color, 1)

            # 3. Draw Fire Bounding Box and Flame-Base Reticle
            if detected and bbox is not None:
                bx, by, bw, bh = bbox
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 69, 255), 2)
                # Mark flame BASE specifically (circle & crosshair at combustion root)
                cv2.circle(frame, (int(ub), int(vb)), 4, (0, 255, 255), -1)
                cv2.putText(frame, f"FLAME BASE", (bx, by + bh + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                cv2.putText(frame, f"CONF: {conf*100:.0f}%", (bx, max(12, by - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 69, 255), 1)

                # Vector from nozzle center to flame base
                cv2.line(frame, (cx, cy), (int(ub), int(vb)), (255, 255, 0), 1)

            # 4. Top Telemetry Bar
            with self.lock:
                telem = dict(self.telemetry)

            state_color = (0, 255, 0) if telem["state"] == "ARMED_IDLE" else \
                          ((0, 0, 255) if telem["state"] in ("FAULT", "DISCONNECTED") else (0, 255, 255))

            cv2.rectangle(frame, (4, 4), (w - 4, 50), (20, 20, 20), -1)
            cv2.rectangle(frame, (4, 4), (w - 4, 50), (60, 60, 60), 1)

            t_text1 = f"STATE: {telem['state']} | FAULT: {telem['fault']} | FPS: {current_fps:.1f} | SESS: {self.session_id[-4:]}"
            cv2.putText(frame, t_text1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, state_color, 1)

            t_text2 = f"INTERLOCK: {telem['interlock']} | BURST: {'ACTIVE' if telem['bursting'] else 'OFF'} (SIM)"
            cv2.putText(frame, t_text2, (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

            # 5. Benchmarking & Calibration HUD (Bottom-Left)
            drop_info = f"DROP: {aim['drop_cm']:.1f}cm" if aim else "DROP: --"
            dist_info = "DIST: 1.0m (CALIB-FIXED, PENDING TOF)"
            cv2.putText(frame, f"{dist_info} | {drop_info}",
                        (8, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 220, 255), 1)

            cv2.putText(frame, f"[A] Arm | [D] Disarm | [SPACE] Fire | [E] E-Stop | [F] Fullscreen | [Q] Quit",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.running = False
                break
            elif key == ord('a') or key == ord('A'):
                print("[CMD] Operator requested ARM")
                self.send_command("arm")
            elif key == ord('d') or key == ord('D'):
                print("[CMD] Operator requested DISARM")
                self.send_command("disarm")
            elif key == ord('e') or key == ord('E'):
                print("[CMD] Operator triggered EMERGENCY STOP")
                self.send_command("estop")
            elif key == ord(' '):
                print("[CMD] Operator authorized DISCHARGE")
                self.send_command("discharge", duration_ms=200)
            elif key == ord('f') or key == ord('F'):
                self.fullscreen = not self.fullscreen
                prop = cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, prop)

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stage 5 Ground Station Client")
    parser.add_argument("--ip", default="192.168.0.113", help="ESP32-S3 IP Address")
    parser.add_argument("--port", type=int, default=80, help="ESP32-S3 Port")
    parser.add_argument("--token", default="DRONE_SECURE_AUTH_STAGE5_2026", help="Configurable Auth Token")
    args = parser.parse_args()

    client = Stage5GroundStation(esp_ip=args.ip, http_port=args.port, auth_token=args.token)
    client.run_gui()
