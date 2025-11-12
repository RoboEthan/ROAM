#!/usr/bin/env python3
import serial, json

UART_PORT = "/dev/serial0"
BAUD = 115200

print(f"[TX] Opening {UART_PORT} @ {BAUD}")
ser = serial.Serial(UART_PORT, BAUD, timeout=1)

def send_obj(obj):
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    ser.write(line.encode("utf-8"))
    ser.flush()
    print(f"[TX] Sent: {line.strip()}")

try:
    while True:
        s = input("Enter T/G or L value (e.g., 120, -80, 0): ").strip().upper()
        if s in ("T", "G"):
            send_obj({"C": s})
        else:
            try:
                L = int(s)
                send_obj({"L": L})
            except ValueError:
                print("Invalid. Type T, G, or an integer (e.g., 120, -80, 0).")
except KeyboardInterrupt:
    print("\n[TX] Bye")
finally:
    ser.close()
