import cv2
import numpy as np
from ctypes import (create_string_buffer, cdll, byref, c_void_p, c_uint32,
                    c_char_p, c_double, c_bool, c_ubyte, POINTER)
from enum import Enum
import warnings
import serial
import time
from datetime import datetime
import csv

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
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_EXPOSURE.value, 120000.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_GAIN.value, 30.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_OFFSET.value, 1.0)
qhyccddll.SetQHYCCDParam(camhandle, CONTROL_ID.CONTROL_USBTRAFFIC.value, 30.0)

# Allocate buffer for 16-bit Bayer raw image
frame_bytes = imageW.value * imageH.value * 2  # 2 bytes per pixel
imgdata_raw16 = (c_ubyte * frame_bytes)()

# Setup serial
ser = serial.Serial('/dev/cu.usbmodem1301', 115200, timeout=1)
value = 32  # number of captures

# CSV log setup
csv_filename = '/Users/ajeems/Downloads/LU/phase shift files/final ver/data3/output.csv'
with open(csv_filename, mode='w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow([
        "Index",
        "Arduino_Reported_Index",
        "Arduino_Reported_Time",
        "Arduino_Latency_ms",
        "Trigger_Recv_PC_Time",
        "Capture_Start_PC_Time",
        "Capture_End_PC_Time",
        "Capture_Duration_sec",
        "Latency_From_Trigger_To_Start_sec"
    ])

    # Capture loop
    for i in range(value):
        print(f"Waiting for trigger {i+1} from Arduino...")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode().strip()

                if line.startswith("T:"):
                    # Parse trigger info
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
                    trigger_recv_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

                    print(f"Trigger {arduino_index} received at {trigger_recv_str} (Arduino time: {arduino_time_str})")

                    capture_start_time = time.time()
                    capture_start_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

                    # Camera exposure
                    camera_exp_start = time.time()
                    qhyccddll.ExpQHYCCDSingleFrame(camhandle)

                    w = c_uint32()
                    h = c_uint32()
                    b = c_uint32()
                    c = c_uint32()

                    ret = qhyccddll.GetQHYCCDSingleFrame(
                        camhandle, byref(w), byref(h), byref(b), byref(c), imgdata_raw16
                    )
                    camera_exp_end = time.time()
                    capture_end_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

                    if ret != 0:
                        print(f"Failed to get frame {i+1}, error code: {ret}")
                    else:
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

                        print(f"[{timestamp}] Captured frame {i+1}: {w.value}x{h.value}, bpp={b.value}, channels={c.value}")

                        # Convert to NumPy array
                        img_np_1d = np.ctypeslib.as_array(imgdata_raw16)
                        img_np = img_np_1d.view(np.uint16).reshape((h.value, w.value))

                        # Demosaic (GBRG pattern)
                        rgb_img = cv2.cvtColor(img_np, cv2.COLOR_BayerGB2RGB)

                        # Extract red channel
                        red_channel = rgb_img[:, :, 2]

                        # Save red channel as PNG
                        filename = f"/Users/ajeems/Downloads/LU/phase shift files/final ver/captured_red_{i+1}.png"
                        cv2.imwrite(filename, red_channel)
                        print(f"[{timestamp}] Red channel image saved: {filename}")

                        # Timing calculations
                        arduino_latency_ms = (capture_start_time - arduino_trigger_time) * 1000
                        capture_duration_sec = camera_exp_end - camera_exp_start
                        latency_from_trigger_to_start = capture_start_time - arduino_trigger_time

                        # Write to CSV
                        csvwriter.writerow([
                            i + 1,
                            arduino_index,
                            arduino_time_str,
                            f"{arduino_latency_ms:.3f}",
                            trigger_recv_str,
                            capture_start_str,
                            capture_end_str,
                            f"{capture_duration_sec:.6f}",
                            f"{latency_from_trigger_to_start:.6f}"
                        ])
                    break
            time.sleep(0.01)

# Cleanup
qhyccddll.CloseQHYCCD(camhandle)
print("Camera closed.")
