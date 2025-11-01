import cv2
import numpy as np
from ctypes import (create_string_buffer, cdll, byref, c_void_p, c_uint32,
                    c_char_p, c_double, c_bool, c_ubyte, POINTER)
from enum import Enum
import warnings
import serial
import time
from datetime import datetime
import os


warnings.filterwarnings("ignore", category=DeprecationWarning)

# Load SDK
qhyccddll = cdll.LoadLibrary(
    '/Users/ajeems/Downloads/LU/phase shift files/dev files/sdk_mac_arm_25.06.16/usr/local/lib/libqhyccd.dylib'
)

# Function prototypes
qhyccddll.GetQHYCCDId.argtypes = [c_uint32, c_char_p]
qhyccddll.OpenQHYCCD.argtypes = [c_char_p]
qhyccddll.OpenQHYCCD.restype = c_void_p
qhyccddll.CloseQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDNumberOfReadModes.argtypes = [c_void_p, POINTER(c_uint32)]
qhyccddll.GetQHYCCDReadModeName.argtypes = [c_void_p, c_uint32, c_char_p]
qhyccddll.GetQHYCCDReadModeResolution.argtypes = [c_void_p, c_uint32, POINTER(c_uint32), POINTER(c_uint32)]
qhyccddll.SetQHYCCDReadMode.argtypes = [c_void_p, c_uint32]
qhyccddll.SetQHYCCDStreamMode.argtypes = [c_void_p, c_uint32]
qhyccddll.InitQHYCCD.argtypes = [c_void_p]
qhyccddll.GetQHYCCDChipInfo.argtypes = [
    c_void_p,
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_double), POINTER(c_double), POINTER(c_uint32)
]
qhyccddll.GetQHYCCDParam.argtypes = [c_void_p, c_uint32]
qhyccddll.GetQHYCCDParam.restype = c_double
qhyccddll.SetQHYCCDParam.argtypes = [c_void_p, c_uint32, c_double]
qhyccddll.SetQHYCCDDebayerOnOff.argtypes = [c_void_p, c_bool]
qhyccddll.SetQHYCCDBinMode.argtypes = [c_void_p, c_uint32, c_uint32]
qhyccddll.SetQHYCCDResolution.argtypes = [c_void_p, c_uint32, c_uint32, c_uint32, c_uint32]
qhyccddll.ExpQHYCCDSingleFrame.argtypes = [c_void_p]
qhyccddll.GetQHYCCDSingleFrame.argtypes = [
    c_void_p,
    POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32),
    POINTER(c_ubyte)
]


# Enum for camera controls
class CONTROL_ID(Enum):
    CONTROL_EXPOSURE = 8
    CONTROL_GAIN = 6
    CONTROL_OFFSET = 7
    CONTROL_USBTRAFFIC = 12
    CONTROL_TRANSFERBIT = 10


# Initialize SDK
ret = qhyccddll.InitQHYCCDResource()
print("InitQHYCCDResource() ret =", ret)

num = qhyccddll.ScanQHYCCD()
print("ScanQHYCCD() num =", num)

camhandle = None
for index in range(num):
    id_buffer = create_string_buffer(40)
    qhyccddll.GetQHYCCDId(index, id_buffer)
    print(f"Camera {index} ID: {id_buffer.value.decode()}")

    camhandle = qhyccddll.OpenQHYCCD(id_buffer)
    if camhandle:
        print("Camera opened.")
        break

if not camhandle:
    raise RuntimeError("No camera could be opened.")

# Basic config
qhyccddll.SetQHYCCDReadMode(camhandle, 0)
qhyccddll.SetQHYCCDStreamMode(camhandle, 0)
qhyccddll.InitQHYCCD(camhandle)

# Disable debayering and use 16-bit output
qhyccddll.SetQHYCCDDebayerOnOff(camhandle, False)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_TRANSFERBIT.value, 16.0)

# Get chip info
chipW = c_double()
chipH = c_double()
imageW = c_uint32()
imageH = c_uint32()
pixelW = c_double()
pixelH = c_double()
imageB = c_uint32()
qhyccddll.GetQHYCCDChipInfo(
    camhandle, byref(chipW), byref(chipH), byref(imageW), byref(imageH),
    byref(pixelW), byref(pixelH), byref(imageB)
)
print(f"Image size: {imageW.value}x{imageH.value}, Bits per pixel: {imageB.value}")

# Set binning and resolution
qhyccddll.SetQHYCCDBinMode(camhandle, 1, 1)
qhyccddll.SetQHYCCDResolution(camhandle, 0, 0, imageW.value, imageH.value)

# Set exposure and gain
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_EXPOSURE.value, 200000.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_GAIN.value, 30.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_USBTRAFFIC.value, 15.0)

# Allocate buffer for 16-bit Bayer raw image
frame_bytes = imageW.value * imageH.value * 2  # 2 bytes per pixel
imgdata_raw16 = (c_ubyte * frame_bytes)()

# --- Setup multiple serial connections ---
ports = [
    '/dev/cu.usbmodem11301',
    '/dev/cu.usbmodem1301',
    '/dev/cu.usbmodem1201',
    '/dev/cu.usbmodem11201'
]

# Create a list (or dictionary) of serial objects
serial_connections = []

for port in ports:
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        serial_connections.append(ser)
        print(f"Connected to {port}")
    except serial.SerialException:
        print(f"Failed to connect to {port}")


value = 68  # number of captures


# --- Base folder where captured runs will be saved ---
base_dir = "/Users/ajeems/Downloads/LU/phase shift files/final ver"
folder_prefix = "sample"

# --- Create a new run folder automatically ---
run_index = 1
while os.path.exists(os.path.join(base_dir, f"{folder_prefix}_{run_index}")):
    run_index += 1

save_dir = os.path.join(base_dir, f"{folder_prefix}_{run_index}")
os.makedirs(save_dir)
print(f"Saving results in: {save_dir}")

try:
    # --- Capture loop ---
    for i in range(value):
        print(f"Waiting for trigger {i+1} from Arduino...")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors='ignore').strip()
                if line.startswith("T:"):
                    parts = line[2:].split(",")
                    if len(parts) != 2:
                        print(f"Invalid trigger format: {line}")
                        continue
                    try:
                        arduino_index = int(parts[0])
                        arduino_time_str = parts[1]
                    except ValueError:
                        print(f"Failed to parse trigger line: {line}")
                        continue

                    arduino_trigger_time = time.time()
                    print(f"Trigger {arduino_index} received (Arduino time: {arduino_time_str})")

                    # --- Camera exposure ---
                    capture_start_time = time.time()
                    qhyccddll.ExpQHYCCDSingleFrame(camhandle)

                    w = c_uint32()
                    h = c_uint32()
                    b = c_uint32()
                    c = c_uint32()

                    try:
                        ret = qhyccddll.GetQHYCCDSingleFrame(
                            camhandle, byref(w), byref(h), byref(b), byref(c), imgdata_raw16
                        )
                    except Exception as e:
                        print(f"Camera exception: {e}")
                        continue

                    if ret != 0:
                        print(f"Failed to get frame {i+1}, error code: {ret}")
                    else:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        print(f"Captured frame {i+1}: {w.value}x{h.value}, bpp={b.value}, channels={c.value}")

                        # --- Convert to NumPy array and demosaic ---
                        img_np_1d = np.ctypeslib.as_array(imgdata_raw16)
                        img_np = img_np_1d.view(np.uint16).reshape((h.value, w.value))
                        rgb_img = cv2.cvtColor(img_np, cv2.COLOR_BayerGB2RGB)
                        red_channel = rgb_img[:, :, 2]

                        # --- Save image in the same folder ---
                        img_filename = os.path.join(save_dir, f"captured_red_{i+1}.png")
                        cv2.imwrite(img_filename, red_channel)
                        print(f"Image saved: {img_filename}")

                        # --- Timing ---
                        capture_end_time = time.time()
                        duration = capture_end_time - capture_start_time
                        latency = capture_start_time - arduino_trigger_time
                        print(
                            f"Timing -> Duration: {duration:.6f}s, Latency: {latency:.6f}s"
                        )

                        time.sleep(0.05)
                    break
            time.sleep(0.05)

finally:
    # --- Cleanup ---
    qhyccddll.CloseQHYCCD(camhandle)
    ser.close()
    print("Camera and serial closed. Images saved in folder: " + save_dir)