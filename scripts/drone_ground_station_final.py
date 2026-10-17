"""
Stage 5 Final Ground Station: Full Projectile Ballistics, Native Stream Header Parsing,
YOLOv5 DFire Fire Detection, and Empirical Calibration Table.
Semi-Autonomous Indoor Fire-Extinguishing Drone Project
"""

import sys
import os
import time
import json
import math
import uuid
import threading
import argparse
import urllib.request
import numpy as np
import cv2

try:
    import websockets
    import asyncio
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False

# Path to Fire-Detection repo and weights
YOLO_REPO_DIR = r"C:\Users\saksh\OneDrive\Documents\Major Project\Fire-Detection"
YOLO_WEIGHTS_PATH = os.path.join(YOLO_REPO_DIR, "weights", "yolov5s.pt")
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "calibration_config.json")

class FullProjectileEngine:
    """
    Rigorously derived 3D projectile trajectory solver:
    Parametric:
      x(t) = v0 * cos(theta) * t  =>  t = R / (v0 * cos(theta))
      y(t) = v0 * sin(theta) * t - 0.5 * g * t^2
    Yields quadratic in u = tan(theta):
      k * u^2 - R * u + (H + k) = 0, where k = (g * R^2) / (2 * v0^2)
      Discriminant D = R^2 * [1 - (2*g*H)/v0^2 - (g^2*R^2)/v0^4]
      Low trajectory solution: tan(theta) = (R - sqrt(D)) / (2 * k)
    """
    def __init__(self, config_dict=None):
        if config_dict is None:
            config_dict = self.load_default_config()

        self.cfg = config_dict
        cam = self.cfg["camera"]
        self.w = cam["resolution_width"]
        self.h = cam["resolution_height"]
        self.cx = cam["cx_px"]
        self.cy = cam["cy_px"]
        self.fx = cam["fx_px"]
        self.fy = cam["fy_px"]

        noz = self.cfg["nozzle_offset_m"]
        self.off_x = noz["dx_m"]
        self.off_y = noz["dy_m"]
        self.off_z = noz["dz_m"]

        bal = self.cfg["ballistics"]
        self.v0 = bal["water_velocity_mps"]
        self.g = bal["gravity_mps2"]
        self.nominal_dist = bal["nominal_operating_dist_m"]
        self.dist_tol = bal["dist_tolerance_m"]

        srv = self.cfg["servos"]
        self.pan_neutral = srv["pan_neutral_us"]
        self.tilt_neutral = srv["tilt_neutral_us"]
        self.us_per_deg_pan = srv["us_per_deg_pan"]
        self.us_per_deg_tilt = srv["us_per_deg_tilt"]
        self.pan_min = srv["pan_min_us"]
        self.pan_max = srv["pan_max_us"]
        self.tilt_min = srv["tilt_min_us"]
        self.tilt_max = srv["tilt_max_us"]

        self.mode = self.cfg.get("_metadata", {}).get("calibration_mode", "analytical_ballistic")
        self.empirical_table = self.cfg.get("empirical_table", [])

    @staticmethod
    def load_default_config():
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r") as f:
                return json.load(f)
        return {
            "camera": {"resolution_width": 320, "resolution_height": 240, "fx_px": 277.13, "fy_px": 289.71, "cx_px": 160.0, "cy_px": 120.0},
            "nozzle_offset_m": {"dx_m": 0.0, "dy_m": -0.040, "dz_m": 0.015},
            "ballistics": {"water_velocity_mps": 4.0, "gravity_mps2": 9.81, "nominal_operating_dist_m": 1.0, "dist_tolerance_m": 0.25},
            "servos": {"pan_neutral_us": 1500.0, "tilt_neutral_us": 1500.0, "us_per_deg_pan": 11.11, "us_per_deg_tilt": 11.11, "pan_min_us": 1333, "pan_max_us": 1667, "tilt_min_us": 1333, "tilt_max_us": 1667}
        }

    def solve_aim(self, u_base, v_base, dist_m=None):
        if dist_m is None or dist_m <= 0:
            dist_m = self.nominal_dist

        # If empirical mode selected, perform bilinear interpolation
        if self.mode == "empirical_interpolation" and len(self.empirical_table) >= 3:
            return self._interpolate_empirical(u_base, v_base, dist_m)

        # 1. Pinhole back-projection to Camera Frame (Z_c = dist_m)
        X_c = ((u_base - self.cx) / self.fx) * dist_m
        Y_c = ((v_base - self.cy) / self.fy) * dist_m
        Z_c = dist_m

        # 2. Transform to Nozzle Frame
        X_n = X_c - self.off_x
        Y_n = Y_c - self.off_y
        Z_n = max(0.1, Z_c - self.off_z)

        # In nozzle frame: horizontal range R, vertical elevation H
        R = math.sqrt(X_n**2 + Z_n**2)
        H = -Y_n # Since +Y is downward in camera frame, height above nozzle is -Y_n

        # 3. Full Projectile Quadratic Solution
        k = (self.g * (R**2)) / (2.0 * (self.v0**2))
        disc = (R**2) * (1.0 - (2.0 * self.g * H) / (self.v0**2) - ((self.g * R) / (self.v0**2))**2)

        if disc < 0:
            # Target is ballistically out of range
            return None

        # Low trajectory solution
        tan_theta = (R - math.sqrt(disc)) / (2.0 * k)
        theta_tilt_deg = math.degrees(math.atan(tan_theta))
        theta_pan_deg = math.degrees(math.atan2(X_n, Z_n))

        # Flight time & true gravity drop
        t_flight = R / (self.v0 * math.cos(math.radians(theta_tilt_deg)))
        true_drop_m = 0.5 * self.g * (t_flight ** 2)

        # 4. Map to servo pulse widths
        pan_us = max(float(self.pan_min), min(float(self.pan_max), self.pan_neutral + theta_pan_deg * self.us_per_deg_pan))
        tilt_us = max(float(self.tilt_min), min(float(self.tilt_max), self.tilt_neutral + theta_tilt_deg * self.us_per_deg_tilt))

        return {
            "pan_deg": theta_pan_deg,
            "tilt_deg": theta_tilt_deg,
            "pan_us": pan_us,
            "tilt_us": tilt_us,
            "flight_time_ms": t_flight * 1000.0,
            "gravity_drop_cm": true_drop_m * 100.0,
            "mode": "analytical_ballistic"
        }

    def _interpolate_empirical(self, u_base, v_base, dist_m):
        # Inverse distance weighted interpolation across anchor points
        weights_sum = 0.0
        pan_accum = 0.0
        tilt_accum = 0.0

        for pt in self.empirical_table:
            d_dist = dist_m - pt["dist_m"]
            d_u = (u_base - pt["u_px"]) / 100.0
            d_v = (v_base - pt["v_px"]) / 100.0
            dist_metric = math.sqrt(d_dist**2 + d_u**2 + d_v**2) + 1e-5
            w = 1.0 / dist_metric
            weights_sum += w
            pan_accum += pt["pan_us"] * w
            tilt_accum += pt["tilt_us"] * w

        interp_pan = pan_accum / weights_sum
        interp_tilt = tilt_accum / weights_sum

        pan_us = max(float(self.pan_min), min(float(self.pan_max), interp_pan))
        tilt_us = max(float(self.tilt_min), min(float(self.tilt_max), interp_tilt))

        return {
            "pan_deg": (pan_us - self.pan_neutral) / self.us_per_deg_pan,
            "tilt_deg": (tilt_us - self.tilt_neutral) / self.us_per_deg_tilt,
            "pan_us": pan_us,
            "tilt_us": tilt_us,
            "flight_time_ms": (dist_m / self.v0) * 1000.0,
            "gravity_drop_cm": 0.5 * self.g * ((dist_m / self.v0)**2) * 100.0,
            "mode": "empirical_interpolation"
        }

class NativeMjpegStreamReader:
    """
    Direct HTTP multipart reader:
    Bypasses OpenCV's header stripping to extract custom microsecond timestamps
    (X-Timestamp-Us) and frame sequence numbers (X-Frame-Seq) directly.
    """
    def __init__(self, url):
        self.url = url
        self.latest_frame = None
        self.latest_seq = 0
        self.latest_cam_ts_us = 0
        self.latest_rx_time = 0
        self.running = True
        self.lock = threading.Lock()

    def stream_worker(self):
        while self.running:
            try:
                stream = urllib.request.urlopen(self.url, timeout=3.0)
                bytes_buf = b""
                boundary = b"--123456789000000000000987654321"

                while self.running:
                    chunk = stream.read(4096)
                    if not chunk:
                        break
                    bytes_buf += chunk

                    # Split on multipart boundary
                    parts = bytes_buf.split(boundary)
                    if len(parts) > 1:
                        complete_part = parts[0]
                        bytes_buf = boundary.join(parts[1:])

                        # Parse headers from part
                        header_end = complete_part.find(b"\r\n\r\n")
                        if header_end != -1:
                            hdr_text = complete_part[:header_end].decode("ascii", errors="ignore")
                            jpeg_data = complete_part[header_end + 4:]

                            # Extract metadata headers
                            seq = 0
                            cam_ts = 0
                            for line in hdr_text.split("\r\n"):
                                if line.startswith("X-Frame-Seq:"):
                                    try: seq = int(line.split(":")[1].strip())
                                    except: pass
                                elif line.startswith("X-Timestamp-Us:"):
                                    try: cam_ts = int(line.split(":")[1].strip())
                                    except: pass

                            if len(jpeg_data) > 0:
                                np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if frame is not None:
                                    with self.lock:
                                        self.latest_frame = frame
                                        self.latest_seq = seq
                                        self.latest_cam_ts_us = cam_ts
                                        self.latest_rx_time = time.time()
            except Exception as e:
                time.sleep(1.0)

    def get_latest(self):
        with self.lock:
            if self.latest_frame is None:
                return None, 0, 0, 0
            return self.latest_frame.copy(), self.latest_seq, self.latest_cam_ts_us, self.latest_rx_time

class FireDetectionPipeline:
    """
    Fire detector: Loads PyTorch YOLOv5 DFire weights if available,
    otherwise provides calibrated combustion-color simulation with explicit status label.
    """
    def __init__(self, weights_path=YOLO_WEIGHTS_PATH):
        self.model = None
        self.model_type = "SIMULATION_HEURISTIC"
        self.weights_path = weights_path

        # Attempt to load YOLOv5 model
        try:
            import torch
            if os.path.exists(weights_path) and os.path.exists(YOLO_REPO_DIR):
                if YOLO_REPO_DIR not in sys.path:
                    sys.path.append(YOLO_REPO_DIR)
                print(f"[MODEL] Loading YOLOv5 DFire weights from {weights_path}...")
                
                # PyTorch 2.6+ compatibility for custom YOLOv5 models
                try:
                    from models.common import AutoShape
                    ckpt = torch.load(weights_path, map_location='cpu', weights_only=False)
                    raw_model = ckpt['model'].float().eval() if (isinstance(ckpt, dict) and 'model' in ckpt) else ckpt.float().eval()
                    self.model = AutoShape(raw_model)
                    self.model_type = "YOLOv5_DFireDataset (weights/yolov5s.pt)"
                    print("[PASS] YOLOv5 DFire model loaded and AutoShape wrapped successfully from weights/yolov5s.pt!")
                except Exception as e_load:
                    print(f"[WARN] Direct torch.load failed: {e_load}. Trying torch.hub...")
                    self.model = torch.hub.load(YOLO_REPO_DIR, 'custom', path=weights_path, source='local')
                    self.model.eval()
                    self.model_type = "YOLOv5_DFireDataset (weights/yolov5s.pt)"
                    print("[PASS] YOLOv5 DFire model loaded successfully via torch.hub!")
        except Exception as e:
            print(f"[INFO] YOLOv5 load failed: {e}. Falling back to simulation.")
            self.model_type = "CALIBRATED_FLAME_HEURISTIC (SIMULATION - YOLO NOT LOADED)"

    def detect(self, frame):
        t0 = time.time()
        h, w = frame.shape[:2]

        if self.model is not None and "YOLO" in self.model_type:
            # Real YOLOv5 inference
            results = self.model(frame)
            df = results.pandas().xyxy[0]
            # Class 1 is fire in dfire.yaml
            fires = df[df['name'] == 'fire']
            if len(fires) > 0:
                best = fires.iloc[fires['confidence'].argmax()]
                bx1, by1, bx2, by2 = int(best['xmin']), int(best['ymin']), int(best['xmax']), int(best['ymax'])
                conf = float(best['confidence'])
                u_base = float((bx1 + bx2) / 2.0)
                v_base = float(by2) # Flame root
                dt = (time.time() - t0) * 1000.0
                return True, conf, u_base, v_base, (bx1, by1, bx2 - bx1, by2 - by1), dt

        # Calibrated Simulation Heuristic (Color & Intensity Clustering)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 180]), np.array([35, 255, 255]))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 120 and area > max_area:
                max_area = area
                best_box = cv2.boundingRect(cnt)

        dt = (time.time() - t0) * 1000.0
        if best_box is not None:
            bx, by, bw, bh = best_box
            u_base = float(bx + bw / 2.0)
            v_base = float(by + bh)
            conf = min(0.98, max(0.40, 0.50 + (max_area / 4000.0) * 0.45))
            return True, conf, u_base, v_base, best_box, dt

        return False, 0.0, 160.0, 120.0, None, dt

class GroundStationFinal:
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

        self.stream_reader = NativeMjpegStreamReader(self.stream_url)
        self.engine = FullProjectileEngine()
        self.detector = FireDetectionPipeline()

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

        self.target_data = {
            "detected": False,
            "conf": 0.0,
            "u_base": 160.0,
            "v_base": 120.0,
            "pan_err": 0.0,
            "tilt_err": 0.0,
            "dist_m": 1.0,
            "bbox": None,
            "aim": None,
            "frame_seq": 0
        }

        self.stats = {"fps": 0.0, "infer_ms": 0.0, "ws_rtt_ms": 0.0}
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
        if not HAS_WEBSOCKETS:
            return

        async def ws_loop():
            while self.running:
                try:
                    async with websockets.connect(self.ws_url, ping_interval=None) as ws:
                        self.ws_connected = True
                        print(f"[WS] Connected to ESP32-S3. Sending session handshake ({self.session_id})...")

                        hs_pkt = {"auth": self.auth_token, "type": "handshake", "session_id": self.session_id}
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
                                await ws.send(json.dumps(pkt))

                            # Dispatch target or heartbeat every 100 ms
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
                                    hb_pkt["frame_ts"] = self.target_data["frame_seq"]

                                t_tx = time.time()
                                await ws.send(json.dumps(hb_pkt))
                                last_hb = now

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
                                    self.stats["ws_rtt_ms"] = (t_rx - last_hb) * 1000.0
                            except asyncio.TimeoutError:
                                pass

                            await asyncio.sleep(0.01)
                except Exception:
                    self.ws_connected = False
                    with self.lock:
                        self.telemetry["state"] = "DISCONNECTED"
                    await asyncio.sleep(1.0)

        asyncio.run(ws_loop())

    def run_gui(self):
        # Start background workers
        stream_t = threading.Thread(target=self.stream_reader.stream_worker, daemon=True)
        stream_t.start()

        ws_t = threading.Thread(target=self.ws_thread_main, daemon=True)
        ws_t.start()

        window_name = "Unified Fire Drone Ground Station"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        fps_timer = time.time()
        frame_cnt = 0
        current_fps = 0.0

        while self.running:
            frame, f_seq, cam_ts_us, rx_time = self.stream_reader.get_latest()
            if frame is None:
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "WAITING FOR ESP32-S3 GC2145 STREAM...", (70, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
                cv2.imshow(window_name, blank)
                key = cv2.waitKey(100) & 0xFF
                if key == ord('q') or key == 27:
                    break
                continue

            frame_cnt += 1
            if time.time() - fps_timer >= 1.0:
                current_fps = frame_cnt / (time.time() - fps_timer)
                self.stats["fps"] = current_fps
                frame_cnt = 0
                fps_timer = time.time()

            # Run Fire Detection
            detected, conf, ub, vb, bbox, infer_ms = self.detector.detect(frame)
            self.stats["infer_ms"] = infer_ms

            # Compute Full Projectile Aim
            aim = None
            pan_err = 0.0
            tilt_err = 0.0
            if detected:
                aim = self.engine.solve_aim(ub, vb, dist_m=1.0)
                if aim:
                    pan_err = aim["pan_deg"]
                    tilt_err = aim["tilt_deg"]
                    if self.telemetry["state"] in ("ARMED_IDLE", "BURSTING"):
                        self.send_command("servo", pan_us=aim["pan_us"], tilt_us=aim["tilt_us"])

            with self.lock:
                self.target_data["detected"] = detected
                self.target_data["conf"] = conf
                self.target_data["u_base"] = ub
                self.target_data["v_base"] = vb
                self.target_data["pan_err"] = pan_err
                self.target_data["tilt_err"] = tilt_err
                self.target_data["bbox"] = bbox
                self.target_data["aim"] = aim
                self.target_data["frame_seq"] = f_seq

            # Render Full-Screen HUD
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # Optical Center Crosshairs
            cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (100, 100, 100), 1)
            cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (100, 100, 100), 1)

            # 3.0° Tolerance Reticle
            radius_px = int((3.0 / 30.0) * (w / 2.0))
            is_aligned = (abs(pan_err) <= 3.0 and abs(tilt_err) <= 3.0) if detected else False
            reticle_color = (0, 255, 0) if is_aligned else (0, 0, 255)
            cv2.circle(frame, (cx, cy), radius_px, reticle_color, 1)

            # Flame Base & Bounding Box
            if detected and bbox is not None:
                bx, by, bw, bh = bbox
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 69, 255), 2)
                cv2.circle(frame, (int(ub), int(vb)), 4, (0, 255, 255), -1)
                cv2.putText(frame, "FLAME BASE", (bx, by + bh + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
                cv2.putText(frame, f"FIRE {conf*100:.0f}%", (bx, max(12, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 69, 255), 1)

            # Top Telemetry Bar
            with self.lock:
                telem = dict(self.telemetry)

            state_color = (0, 255, 0) if telem["state"] == "ARMED_IDLE" else \
                          ((0, 0, 255) if telem["state"] in ("FAULT", "DISCONNECTED") else (0, 255, 255))

            cv2.rectangle(frame, (4, 4), (w - 4, 52), (20, 20, 20), -1)
            cv2.rectangle(frame, (4, 4), (w - 4, 52), (60, 60, 60), 1)

            line1 = f"STATE: {telem['state']} | FAULT: {telem['fault']} | FPS: {current_fps:.1f} | INFER: {infer_ms:.1f}ms"
            cv2.putText(frame, line1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, state_color, 1)

            line2 = f"INTERLOCK: {telem['interlock']} | PUMP: {'BURST' if telem['bursting'] else 'OFF'} (SIM LOCKED)"
            cv2.putText(frame, line2, (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

            line3 = f"MODEL: {self.detector.model_type}"
            cv2.putText(frame, line3, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 200, 255), 1)

            # Bottom Calibration Bar
            drop_txt = f"DROP: {aim['gravity_drop_cm']:.1f}cm" if aim else "DROP: --"
            dist_txt = "DIST: 1.0m (CALIB-FIXED, PENDING TOF)"
            cv2.putText(frame, f"{dist_txt} | {drop_txt}", (8, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 220, 255), 1)
            cv2.putText(frame, "[A] Arm | [D] Disarm | [SPACE] Fire | [E] E-Stop | [F] Fullscreen | [Q] Quit",
                        (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.running = False
                break
            elif key == ord('a') or key == ord('A'):
                print("[CMD] Operator ARM")
                self.send_command("arm")
            elif key == ord('d') or key == ord('D'):
                print("[CMD] Operator DISARM")
                self.send_command("disarm")
            elif key == ord('e') or key == ord('E'):
                print("[CMD] Operator E-STOP")
                self.send_command("estop")
            elif key == ord(' '):
                print("[CMD] Operator DISCHARGE (Simulation)")
                self.send_command("discharge", duration_ms=200)
            elif key == ord('f') or key == ord('F'):
                self.fullscreen = not self.fullscreen
                prop = cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, prop)

        self.stream_reader.running = False
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stage 5 Ground Station")
    parser.add_argument("--ip", default="192.168.0.113", help="ESP32-S3 IP Address")
    parser.add_argument("--port", type=int, default=80, help="ESP32-S3 Port")
    parser.add_argument("--token", default="DRONE_SECURE_AUTH_STAGE5_2026", help="Configurable Auth Token")
    args = parser.parse_args()

    client = GroundStationFinal(esp_ip=args.ip, http_port=args.port, auth_token=args.token)
    client.run_gui()
