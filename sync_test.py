import serial
import time
import cv2
import os
import numpy as np
from datetime import datetime

# Updated ports: Prioritize your cu port
ports = [
    '/dev/cu.usbmodem1301',  # Your actual port first
    '/dev/cu.usbmodem11301',
    '/dev/cu.usbmodem1401',
    '/dev/cu.usbmodem1201',
    '/dev/cu.usbmodem11201',
    '/dev/cu.usbmodem11301',
    '/dev/cu.usbmodem1201',
    '/dev/cu.usbmodem11201'
]

serial_connections = []
ser = None
for port in ports:
    try:
        ser_temp = serial.Serial(port, 115200, timeout=1)
        serial_connections.append(ser_temp)
        ser = ser_temp
        print(f"Connected to Arduino on {port}")
        break
    except serial.SerialException:
        print(f"Failed to connect to {port}")

if ser is None:
    raise RuntimeError("No Arduino serial port could be opened.")

time.sleep(5)  # Wait for Arduino startup blink demo

BIG_RING_SIZE = 24
value = 24  # number of cycles
BLINK_DURATION = 1000  # ms hold per blink (add to command)
CAPTURE_SETTLE_DELAY = 0.05  # 50ms delay after LED ON for camera to capture peak (increased)
NUM_CAPTURES_PER_BLINK = 3  # Capture multiple frames during hold to ensure one hits lit period

# --- Camera setup: Default Mac camera (0 = built-in) ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Cannot open default Mac camera")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Try manual exposure for better sync (Mac AVFoundation backend)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode (0.25 = manual on some backends)
cap.set(cv2.CAP_PROP_EXPOSURE, 0.1)  # 0.1s exposure time (adjust 0.05-0.5 for brighter lit capture)

print("Camera properties set: Manual exposure 0.1s")

# --- Base folder where captured runs will be saved ---
base_dir = "/Users/ajeems/Downloads/LU/phase shift files/final ver"
folder_prefix = "sample"

# --- Create a new run folder automatically ---
run_index = 1
while os.path.exists(os.path.join(base_dir, f"{folder_prefix}_{run_index}")):
    run_index += 1

save_dir = os.path.join(base_dir, f"{folder_prefix}_{run_index}")
os.makedirs(save_dir)
print(f"Saving images in: {save_dir}")

try:
    # --- LED Sync + Camera Capture Loop ---
    for i in range(value):
        print(f"Starting cycle {i+1}/{value}...")

        # Step 1: Signal Arduino LED ON with duration (cycle 0-23)
        led_index = i % BIG_RING_SIZE
        led_start_time = time.time()
        ser.write(f'LIGHT:{led_index}:{BLINK_DURATION}\n'.encode())
        ser.flush()
        print(f"LED {led_index} ON for {BLINK_DURATION}ms sent at {led_start_time:.6f}")

        # Step 2: Settle time for LED max + camera sync
        time.sleep(0.001 + CAPTURE_SETTLE_DELAY)  # 1ms LED + 50ms for camera

        # Step 3: Capture multiple images during hold, pick brightest (for lit LED)
        capture_start_time = time.time()
        frames = []
        brightnesses = []
        for j in range(NUM_CAPTURES_PER_BLINK):
            ret, frame = cap.read()
            if ret:
                # Compute mean brightness (grayscale for simplicity)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                mean_bright = np.mean(gray)
                frames.append(frame)
                brightnesses.append(mean_bright)
                print(f"  Sub-capture {j+1}: Mean brightness {mean_bright:.1f}")
            else:
                print(f"  Failed sub-capture {j+1}")
            time.sleep(0.1)  # 100ms between sub-captures (total ~300ms spread)

        # Pick and save the brightest frame (most likely during lit period)
        if frames:
            brightest_idx = np.argmax(brightnesses)
            best_frame = frames[brightest_idx]
            best_bright = brightnesses[brightest_idx]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_filename = os.path.join(save_dir, f"captured_{i+1}_{timestamp}_bright{best_bright:.0f}.png")
            cv2.imwrite(img_filename, best_frame)
            print(f"  Best image saved: {img_filename} (brightness {best_bright:.1f})")
        else:
            print(f"  No frames captured for cycle {i+1}")

        capture_end_time = time.time()
        print(f"Capture complete at {capture_end_time:.6f}")

        # --- Timing (jitter-focused) ---
        duration = capture_end_time - capture_start_time
        led_to_capture_latency = capture_start_time - led_start_time
        print(
            f"Timing -> Capture Duration: {duration:.6f}s, LED-ON to Capture Latency: {led_to_capture_latency:.6f}s"
        )

        # Step 4: Wait remaining hold time (total ~1s)
        remaining_hold = BLINK_DURATION / 1000.0 - (capture_end_time - led_start_time)
        if remaining_hold > 0:
            time.sleep(remaining_hold)

        # No explicit OFF—Arduino auto-clears after duration
        # Optional: Brief pause between cycles
        time.sleep(0.05)

    print("Sequence complete—all captures with LED sync!")

except Exception as e:
    print(f"Error during sequence: {e}")
    if ser:
        ser.write(b'OFF\n')  # Emergency OFF
        ser.flush()

finally:
    # --- Cleanup ---
    cap.release()
    cv2.destroyAllWindows()
    for s in serial_connections:
        s.close()
    print(f"Camera and serial closed. Images saved in: {save_dir}")