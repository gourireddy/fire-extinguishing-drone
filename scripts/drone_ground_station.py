"""
Laptop Ground Station & Fire-Tracking Integration Client
Semi-Autonomous Indoor Fire-Extinguishing Drone Project
Target: ESP32-S3 Unified System (MJPEG Video Stream + WebSocket Telemetry)
"""

import sys
import time
import json
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

AUTH_TOKEN = "FIRE_DRONE_AUTH_KEY_2026"

class GroundStationClient:
    def __init__(self, esp_ip="192.168.0.113", http_port=80):
        self.esp_ip = esp_ip
        self.http_port = http_port
        self.stream_url = f"http://{esp_ip}:{http_port}/stream"
        self.ws_url = f"ws://{esp_ip}:{http_port}/ws"

        self.running = True
        self.seq_num = 1
        self.lock = threading.Lock()

        # Telemetry received from drone
        self.telemetry = {
            "state": "DISCONNECTED",
            "fault": "NONE",
            "interlock": "NO_CONNECTION",
            "pan_us": 1500,
            "tilt_us": 1500,
            "bursting": False,
            "simulated": True,
            "last_rx_time": 0
        }

        # Latest vision target data
        self.target_data = {
            "detected": False,
            "conf": 0.0,
            "pan_err": 0.0,
            "tilt_err": 0.0,
            "dist_m": 1.0,
            "bbox": None
        }

        # Command queue for WebSocket thread
        self.pending_commands = []
        self.ws_connected = False
        self.fullscreen = False

    def send_command(self, cmd_type, **kwargs):
        with self.lock:
            self.seq_num += 1
            pkt = {
                "auth": AUTH_TOKEN,
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
                        print(f"[WS] Connected to ESP32-S3 at {self.ws_url}")
                        last_hb = time.time()

                        while self.running:
                            # 1. Send pending commands or periodic heartbeats
                            now = time.time()
                            to_send = []
                            with self.lock:
                                if self.pending_commands:
                                    to_send = list(self.pending_commands)
                                    self.pending_commands.clear()

                            for pkt in to_send:
                                await ws.send(json.dumps(pkt))

                            # Send heartbeat every 120 ms to keep watchdog satisfied
                            if now - last_hb > 0.120:
                                self.seq_num += 1
                                hb_pkt = {
                                    "auth": AUTH_TOKEN,
                                    "seq": self.seq_num,
                                    "ts": int(now * 1000),
                                    "type": "target" if self.target_data["detected"] else "heartbeat"
                                }
                                if self.target_data["detected"]:
                                    hb_pkt["detected"] = True
                                    hb_pkt["conf"] = self.target_data["conf"]
                                    hb_pkt["pan_err"] = self.target_data["pan_err"]
                                    hb_pkt["tilt_err"] = self.target_data["tilt_err"]
                                    hb_pkt["dist_m"] = self.target_data["dist_m"]
                                await ws.send(json.dumps(hb_pkt))
                                last_hb = now

                            # 2. Receive telemetry responses (non-blocking with short timeout)
                            try:
                                msg = await asyncio.wait_for(ws.recv(), timeout=0.05)
                                data = json.loads(msg)
                                with self.lock:
                                    self.telemetry["state"] = data.get("state", "UNKNOWN")
                                    self.telemetry["fault"] = data.get("fault", "NONE")
                                    self.telemetry["interlock"] = data.get("interlock", "UNKNOWN")
                                    self.telemetry["pan_us"] = data.get("pan_us", 1500)
                                    self.telemetry["tilt_us"] = data.get("tilt_us", 1500)
                                    self.telemetry["bursting"] = data.get("bursting", False)
                                    self.telemetry["simulated"] = data.get("simulated", True)
                                    self.telemetry["last_rx_time"] = time.time()
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
        Lightweight Fire Detection Engine:
        Analyzes HSV color space for high-intensity flame chromaticities (Hue: 0-30, High Sat & Val)
        with spatial clustering, returning bounding box, center errors, and confidence.
        Can be substituted with full YOLOv8 / YOLOv11 ONNX weights.
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Flame color range (Orange/Yellow/Red combustion core)
        lower_flame = np.array([0, 120, 180])
        upper_flame = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_flame, upper_flame)

        # Morphological filtering to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        max_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 150: # filter speckles
                if area > max_area:
                    max_area = area
                    best_box = cv2.boundingRect(cnt)

        if best_box is not None:
            bx, by, bw, bh = best_box
            cx = bx + bw / 2.0
            cy = by + bh / 2.0

            # Compute angular errors relative to optical center
            # Assumed GC2145 QVGA FOV: HFOV ≈ 60 deg, VFOV ≈ 45 deg
            norm_x = (cx - (w / 2.0)) / (w / 2.0)
            norm_y = (cy - (h / 2.0)) / (h / 2.0)
            pan_err_deg = norm_x * 30.0
            tilt_err_deg = -norm_y * 22.5

            # Confidence estimated from flame saturation and area
            conf = min(0.98, max(0.40, 0.50 + (max_area / 5000.0) * 0.45))

            # Approximate standoff distance (inverse square root of bbox height)
            dist_m = max(0.4, min(3.0, 120.0 / float(max(bh, 10))))

            return True, conf, pan_err_deg, tilt_err_deg, dist_m, (bx, by, bw, bh)

        return False, 0.0, 0.0, 0.0, 1.0, None

    def run_gui(self):
        """Main OpenCV Video Display & Operator HUD"""
        print(f"[STREAM] Opening video capture at {self.stream_url}...")
        cap = cv2.VideoCapture(self.stream_url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Start WebSocket thread
        ws_thread = threading.Thread(target=self.ws_thread_main, daemon=True)
        ws_thread.start()

        window_name = "ESP32-S3 Fire Drone Ground Station"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        fps_timer = time.time()
        frame_count = 0
        current_fps = 0.0

        while self.running:
            ret, frame = cap.read()
            if not ret or frame is None:
                # Stream drop or buffering -> Show reconnecting screen
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "CONNECTING TO DRONE CAMERA STREAM...", (80, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
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
                frame_count = 0
                fps_timer = time.time()

            # Process Fire Detection
            detected, conf, pan_err, tilt_err, dist_m, bbox = self.detect_fire(frame)
            with self.lock:
                self.target_data["detected"] = detected
                self.target_data["conf"] = conf
                self.target_data["pan_err"] = pan_err
                self.target_data["tilt_err"] = tilt_err
                self.target_data["dist_m"] = dist_m
                self.target_data["bbox"] = bbox

            # Render HUD Overlay
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # Optical Center Crosshairs
            cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (100, 100, 100), 1)
            cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (100, 100, 100), 1)

            # Draw 3.0° Alignment Tolerance Circle
            # 3 deg / 30 deg * (w / 2) ≈ radius in pixels
            radius_px = int((3.0 / 30.0) * (w / 2.0))
            is_aligned = abs(pan_err) <= 3.0 and abs(tilt_err) <= 3.0 if detected else False
            reticle_color = (0, 255, 0) if is_aligned else (0, 0, 255)
            cv2.circle(frame, (cx, cy), radius_px, reticle_color, 1)

            # Draw Fire Bounding Box & Vector
            if detected and bbox is not None:
                bx, by, bw, bh = bbox
                box_color = (0, 69, 255) # Deep orange-red
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), box_color, 2)
                cv2.putText(frame, f"FIRE {conf*100:.1f}%", (bx, max(15, by - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1)
                # Line from center to flame
                cv2.line(frame, (cx, cy), (int(bx + bw / 2), int(by + bh / 2)), (0, 255, 255), 1)

            # Top Telemetry Bar
            with self.lock:
                telem = dict(self.telemetry)

            state_color = (0, 255, 0) if telem["state"] == "ARMED_IDLE" else \
                          ((0, 0, 255) if telem["state"] in ("FAULT", "DISCONNECTED") else (0, 255, 255))
            
            # HUD Background Box
            cv2.rectangle(frame, (5, 5), (w - 5, 45), (20, 20, 20), -1)
            cv2.rectangle(frame, (5, 5), (w - 5, 45), (60, 60, 60), 1)

            hud_text1 = f"STATUS: {telem['state']} | FAULT: {telem['fault']} | FPS: {current_fps:.1f}"
            cv2.putText(frame, hud_text1, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, state_color, 1)

            hud_text2 = f"INTERLOCK: {telem['interlock']} | BURST: {'ACTIVE' if telem['bursting'] else 'OFF'} (SIM)"
            cv2.putText(frame, hud_text2, (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)

            # Bottom Status Info
            cv2.putText(frame, f"[A] Arm | [D] Disarm | [SPACE] Discharge | [E] E-Stop | [F] Fullscreen | [Q] Quit",
                        (10, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1)

            cv2.imshow(window_name, frame)

            # Key handling
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
    parser = argparse.ArgumentParser(description="Fire Drone Ground Station Client")
    parser.add_argument("--ip", default="192.168.0.113", help="ESP32-S3 IP Address")
    parser.add_argument("--port", type=int, default=80, help="ESP32-S3 HTTP/WS Port")
    args = parser.parse_args()

    client = GroundStationClient(esp_ip=args.ip, http_port=args.port)
    client.run_gui()
