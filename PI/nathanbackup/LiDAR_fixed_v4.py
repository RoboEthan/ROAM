#!/usr/bin/env python3
import math
import time
import ydlidar

# ----------------------------
# CONFIGURATION
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUD = 128000
HEIGHT_M = 0.292          # LiDAR height = 11.5 in
TILT_DEG = 12.0           # tilt downward from horizontal
TILT_RAD = math.radians(TILT_DEG)
TOL = 0.08                # 8 cm tolerance for ground detection
SCAN_TIME = 5.0           # seconds per scan window
# ----------------------------

ydlidar.os_init()
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, PORT)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 5.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 10.0)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)
laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

if not laser.initialize():
    print("[ERROR] Failed to initialize LiDAR")
    exit(1)
if not laser.turnOn():
    print("[ERROR] Failed to start LiDAR scanning")
    exit(1)

print("[LiDAR] Scanning for {:.1f}s to determine FOV...".format(SCAN_TIME))
scan = ydlidar.LaserScan()
start = time.time()
all_points = []

def is_ground_hit(angle_deg, r):
    """Return True if this beam likely hits the ground"""
    theta = math.radians(angle_deg)
    drop = r * math.sin(TILT_RAD) * math.cos(theta)
    return abs(drop - HEIGHT_M) <= TOL

try:
    while time.time() - start < SCAN_TIME:
        if not laser.doProcessSimple(scan):
            continue
        for p in scan.points:
            angle = p.angle if hasattr(p, "angle") else 0.0
            if abs(angle) > math.pi:
                continue
            angle_deg = math.degrees(angle)
            r = p.range
            if r <= 0.05 or r > 10.0:
                continue
            all_points.append((angle_deg, r))
finally:
    laser.turnOff()
    laser.disconnecting()

print(f"[LiDAR] {len(all_points)} points captured total")

# --- find ground points ---
ground_points = [(a, r) for a, r in all_points if is_ground_hit(a, r)]

if not ground_points:
    print("[WARN] No ground-like points detected — check tilt or height values")
    exit(0)

angles = [a for a, _ in ground_points]
fov_min, fov_max = min(angles), max(angles)
fov_width = fov_max - fov_min

print(f"[RESULT] Estimated visible ground FOV: {fov_width:.1f}° "
      f"(min={fov_min:.1f}°, max={fov_max:.1f}°)")

# --- Print a subset of ground points (every ~10 degrees) ---
print("\n[Ground hits sample: angle°, distance m]")
step = 10
for deg in range(int(fov_min), int(fov_max)+1, step):
    # pick the point closest to that degree
    closest = min(ground_points, key=lambda p: abs(p[0]-deg))
    print(f" {closest[0]:6.2f}°  →  {closest[1]:.3f} m")

print("\n[LiDAR] Done.")
