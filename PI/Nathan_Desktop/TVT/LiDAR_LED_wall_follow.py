#!/usr/bin/env python3
import time
import csv
from datetime import datetime
from gpiozero import LED
import ydlidar

# ===================== GPIO SETUP =====================
LEFT_PIN = 21      # GPIO for LEFT indicator LED
RIGHT_PIN = 20     # GPIO for RIGHT indicator LED
left_led = LED(LEFT_PIN)
right_led = LED(RIGHT_PIN)

# ===================== LOGGING SETUP =====================
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"lidar_log_{timestamp}.csv"
csv_file = open(csv_filename, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "LeftAvg_m", "RightAvg_m", "Diff_m", "Action"])

# ===================== LIDAR SETUP =====================
ydlidar.os_init()
ports = ydlidar.lidarPortList()
port = "/dev/ttyUSB0"
for _, value in ports.items():
    port = value
    print(f"[LiDAR] Found on: {port}")

laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000)
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
    print("[ERROR] Lidar initialize() failed")
    exit(1)
if not laser.turnOn():
    print("[ERROR] Lidar turnOn() failed")
    exit(1)

print("[LiDAR] Running sidewalk-follow mode...")
scan = ydlidar.LaserScan()

# ===================== MAIN LOOP =====================
try:
    while True:
        if laser.doProcessSimple(scan):
            # Extract ±45° region (front FOV)
            points = [(p.angle, p.range) for p in scan.points if -45 <= p.angle <= 45 and p.range > 0.1]
            if not points:
                continue

            # Split into left/right half sectors
            left_pts = [d for a, d in points if -45 <= a < 0]
            right_pts = [d for a, d in points if 0 <= a <= 45]

            if not left_pts or not right_pts:
                continue

            left_avg = sum(left_pts) / len(left_pts)
            right_avg = sum(right_pts) / len(right_pts)
            diff = right_avg - left_avg

            # Determine steering correction
            if diff > 0.1:
                # Object closer on left, steer RIGHT
                right_led.on()
                left_led.off()
                action = "RIGHT"
            elif diff < -0.1:
                # Object closer on right, steer LEFT
                left_led.on()
                right_led.off()
                action = "LEFT"
            else:
                left_led.off()
                right_led.off()
                action = "CENTER"

            print(f"[{action:6}] Left={left_avg:.2f}m  Right={right_avg:.2f}m  Diff={diff:+.3f}m")

            # Log to CSV
            csv_writer.writerow([datetime.now().isoformat(), f"{left_avg:.3f}", f"{right_avg:.3f}", f"{diff:.3f}", action])
            csv_file.flush()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[STOP] Keyboard interrupt, shutting down...")
finally:
    # Clean shutdown
    laser.turnOff()
    laser.disconnecting()
    left_led.off()
    right_led.off()
    csv_file.close()
    print(f"[LiDAR] Stopped. Data saved to {csv_filename}")
