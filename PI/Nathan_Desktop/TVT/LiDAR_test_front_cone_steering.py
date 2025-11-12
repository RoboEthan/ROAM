#!/usr/bin/env python3
import time
import ydlidar
import math
import statistics

# === CONFIG ===
PORT = "/dev/ttyUSB0"
BAUDRATE = 128000
MIN_RANGE = 0.05
MAX_RANGE = 10.0
FRONT_CONE = 60  # degrees left/right
UPDATE_DELAY = 0.05  # seconds between scans

# Left/right cone definitions (inside ±60° forward cone)
RIGHT_MIN, RIGHT_MAX = -FRONT_CONE, -20
LEFT_MIN,  LEFT_MAX  =  20, FRONT_CONE
CENTER_MIN, CENTER_MAX = -20, 20

# === INIT LIDAR ===
ydlidar.os_init()
ports = ydlidar.lidarPortList()
for _, val in ports.items():
    PORT = val
    print(f"[LiDAR] Found: {PORT}")

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUDRATE)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 5.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, MAX_RANGE)
laser.setlidaropt(ydlidar.LidarPropMinRange, MIN_RANGE)
laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

if not laser.initialize():
    print("[ERROR] initialize() failed")
    exit(1)
if not laser.turnOn():
    print("[ERROR] turnOn() failed")
    exit(1)

print("[LiDAR] Tracking grass/sidewalk edges... press Ctrl+C to stop")
scan = ydlidar.LaserScan()

try:
    while True:
        if not laser.doProcessSimple(scan):
            time.sleep(UPDATE_DELAY)
            continue

        # Collect valid points
        valid_points = [
            (math.degrees(p.angle), p.range)
            for p in scan.points
            if MIN_RANGE < p.range < MAX_RANGE
        ]

        # Split into sectors
        left = [r for a, r in valid_points if LEFT_MIN <= a <= LEFT_MAX]
        right = [r for a, r in valid_points if RIGHT_MIN <= a <= RIGHT_MAX]
        center = [r for a, r in valid_points if CENTER_MIN <= a <= CENTER_MAX]

        # Compute average distances
        avg_left = statistics.mean(left) if left else None
        avg_right = statistics.mean(right) if right else None
        avg_center = statistics.mean(center) if center else None

        # Compute steering correction (positive = steer right)
        if avg_left is not None and avg_right is not None:
            steer_error = avg_left - avg_right
        else:
            steer_error = 0.0

        # Safe formatting helper
        def fmt(x):
            return f"{x:.2f}" if x is not None else "--"

        # Print live tracking info
        print(
            f"L: {fmt(avg_left)} m | "
            f"C: {fmt(avg_center)} m | "
            f"R: {fmt(avg_right)} m | "
            f"Δ={steer_error:+.2f}"
        )


        time.sleep(UPDATE_DELAY)

except KeyboardInterrupt:
    print("\n[LiDAR] Stopping...")

laser.turnOff()
laser.disconnecting()
print("[LiDAR] Stopped.")
