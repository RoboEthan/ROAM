import serial
import time
import sys

# === CONFIG ===
PORT = '/dev/serial0'
BAUD = 115200
TIMEOUT = 1
TEST_MESSAGE = "ROAM UART DEBUG TEST\n"

print("[INFO] Initializing UART...")
try:
    uart = serial.Serial(PORT, baudrate=BAUD, timeout=TIMEOUT)
    time.sleep(2)  # Let port settle

    print(f"[INFO] Port opened: {uart.name}")
    print(f"[INFO] Baudrate: {BAUD}")
    print(f"[INFO] Sending message: {TEST_MESSAGE.strip()}")

    uart.reset_input_buffer()
    uart.reset_output_buffer()

    # Send message
    bytes_written = uart.write(TEST_MESSAGE.encode())
    uart.flush()  # Force write from buffer
    print(f"[DEBUG] Bytes written: {bytes_written}")

    time.sleep(0.2)  # Give time for loopback

    # Check if data is available
    waiting = uart.in_waiting
    print(f"[DEBUG] Bytes waiting in input buffer: {waiting}")

    if waiting > 0:
        response = uart.readline().decode(errors='replace').strip()
        print(f"[SUCCESS] Loopback received: {response}")
        if response == TEST_MESSAGE.strip():
            print("[RESULT] Exact match received ✅")
        else:
            print("[WARNING] Mismatched response ⚠️")
    else:
        print("[ERROR] No data received — is GPIO14 (TX) connected to GPIO15 (RX)?")

except serial.SerialException as e:
    print("[FATAL] Serial port error:", e)
    sys.exit(1)
except Exception as ex:
    print("[FATAL] Unexpected error:", ex)
    sys.exit(1)
finally:
    if 'uart' in locals() and uart.is_open:
        uart.close()
        print("[INFO] UART port closed.")
