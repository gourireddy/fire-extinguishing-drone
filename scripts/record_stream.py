"""
ESP32 Camera Live Stream Viewer & Recorder with YOLO Integration Hook
Designed for Vikas R&D Quadcopter Fire-Extinguishing Drone Project.
"""

import cv2
import time
import os
import sys
from datetime import datetime

# ================= Configuration =================
DEFAULT_STREAM_URL = "http://192.168.0.113/stream"
RECORDINGS_DIR = os.path.join(os.path.dirname(__file__), "..", "recordings")
SNAPSHOTS_DIR = os.path.join(RECORDINGS_DIR, "snapshots")

os.makedirs(RECORDINGS_DIR, exist_ok=True)
os.makedirs(SNAPSHOTS_DIR, exist_ok=True)


def process_frame_yolo(frame):
    """
    Placeholder for future YOLO / OpenCV fire-detection model.
    Pass the BGR frame here to perform inference, draw bounding boxes,
    or calculate aiming coordinates for the nozzle servo.
    """
    # [Future Hook: Insert model inference here]
    # detections = fire_model.predict(frame)
    # return annotated_frame, detections
    return frame, None


def main():
    stream_url = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STREAM_URL
    print("=" * 60)
    print("  ESP32-S3 Drone Camera Live Stream Viewer & Recorder")
    print("=" * 60)
    print(f"Connecting to stream: {stream_url}")
    print("\nControls:")
    print("  [R] - Start / Stop Video Recording")
    print("  [S] - Save Snapshot Image")
    print("  [Q] or [ESC] - Quit")
    print("=" * 60)

    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    is_recording = False
    video_writer = None
    fps_time = time.time()
    fps = 0
    frame_count = 0

    cv2.namedWindow("ESP32 Drone Feed", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Stream disconnected or waiting for frames... retrying in 1s")
            time.sleep(1)
            cap.release()
            cap = cv2.VideoCapture(stream_url)
            continue

        h, w = frame.shape[:2]

        # Calculate display FPS
        frame_count += 1
        curr_time = time.time()
        if curr_time - fps_time >= 1.0:
            fps = frame_count / (curr_time - fps_time)
            frame_count = 0
            fps_time = curr_time

        # If recording, write raw camera frame (before HUD text)
        if is_recording and video_writer is not None:
            video_writer.write(frame)

        # Pass frame to YOLO hook
        processed_frame, _ = process_frame_yolo(frame)
        display_frame = processed_frame.copy()

        # Render On-Screen Display (HUD)
        # Background bar for HUD
        cv2.rectangle(display_frame, (0, 0), (w, 35), (20, 20, 20), -1)

        # FPS & Resolution
        hud_text = f"FPS: {fps:.1f} | Res: {w}x{h}"
        cv2.putText(display_frame, hud_text, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

        # Recording Status Badge
        if is_recording:
            # Flashing red circle
            if int(time.time() * 2) % 2 == 0:
                cv2.circle(display_frame, (w - 100, 18), 7, (0, 0, 255), -1)
            cv2.putText(display_frame, "REC", (w - 85, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.circle(display_frame, (w - 100, 18), 7, (0, 200, 0), -1)
            cv2.putText(display_frame, "LIVE", (w - 85, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("ESP32 Drone Feed", display_frame)

        key = cv2.waitKey(1) & 0xFF

        # [Q] or [ESC] to exit
        if key == ord('q') or key == 27:
            break

        # [R] to toggle recording
        elif key == ord('r') or key == ord('R'):
            is_recording = not is_recording
            if is_recording:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(RECORDINGS_DIR, f"drone_feed_{timestamp}.mp4")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                target_fps = fps if fps > 5 else 15.0
                video_writer = cv2.VideoWriter(filename, fourcc, target_fps, (w, h))
                print(f"[INFO] Started recording: {filename}")
            else:
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None
                print("[INFO] Recording stopped and saved.")

        # [S] to save snapshot
        elif key == ord('s') or key == ord('S'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(SNAPSHOTS_DIR, f"snapshot_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            print(f"[INFO] Snapshot saved: {filename}")

    # Cleanup
    if video_writer is not None:
        video_writer.release()
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Application closed.")


if __name__ == "__main__":
    main()
