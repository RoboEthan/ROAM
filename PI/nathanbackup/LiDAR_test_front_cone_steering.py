#!/usr/bin/env python3
import time
import ydlidar
import math

# === CONFIG ===
PORT = "/dev/ttyUSB0"
BAUDRATE = 128000
MIN_RANGE = 0.05
MAX_RANGE = 10.0
TARGET_ANGLES = list(range(-60, 70, 10))  # -60 to 60 in steps of 10

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

print("[LiDAR] Scanning target angles... press Ctrl+C to stop")
scan = ydlidar.LaserScan()

try:
    while True:
        if laser.doProcessSimple(scan):
            # Dictionary to store closest point per target angle
            closest_points = {angle: None for angle in TARGET_ANGLES}
            for p in scan.points:
                if MIN_RANGE < p.range < MAX_RANGE:
                    angle_deg = math.degrees(p.angle)
                    # Find nearest target angle
                    nearest_target = min(TARGET_ANGLES, key=lambda x: abs(x - angle_deg))
                    # Store if empty or closer than previous
                    if (closest_points[nearest_target] is None or 
                        abs(angle_deg - nearest_target) < abs(math.degrees(closest_points[nearest_target].angle) - nearest_target)):
                        closest_points[nearest_target] = p

            # Print results
            for angle in TARGET_ANGLES:
                p = closest_points[angle]
                if p is not None:
                    print(f"Angle: {angle:+3d}°, Distance: {p.range:.3f} m")
            print("---")
        else:
            time.sleep(0.01)
except KeyboardInterrupt:
    print("\n[LiDAR] Stopping scan...")

laser.turnOff()
laser.disconnecting()
print("[LiDAR] Stopped.")
