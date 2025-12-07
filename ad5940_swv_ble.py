import sys
if sys.platform.startswith("win"):
    import asyncio
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

import asyncio
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from collections import deque
import re
import csv
import datetime
import threading
import time
import os

# --- CONFIGURATION ---
ESP32_NAME = "ESP32_AMPEROMETRIC"  # your ESP32's advertised name
SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
CHARACTERISTIC_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

WINDOW_SIZE = 600
data_buffer = deque(maxlen=WINDOW_SIZE)

CSV_FILENAME = r"C:\Users\ccwur\Downloads\12_5_swv_test.csv"
# Create CSV only if missing
file_exists = os.path.isfile(CSV_FILENAME)

with open(CSV_FILENAME, "a", newline="") as f:
    writer = csv.writer(f)
    if not file_exists:
        writer.writerow(["timestamp", "index", "current_A"])

# --- PLOTTING THREAD ---
def plot_thread():
    plt.ion()
    fig, ax = plt.subplots()
    (line,) = ax.plot([], [], lw=2)
    ax.set_xlabel("Index")
    ax.set_ylabel("Current (A)")
    ax.set_title("Live Current Data from ESP32 BLE")
    ax.grid(True)

    while True:
        if len(data_buffer) > 0:
            xs = [p[0] for p in data_buffer]  # indexes
            ys = [p[1] for p in data_buffer]  # values

            line.set_data(xs, ys)

            ax.set_xlim(min(xs), max(xs))
            ymin, ymax = min(ys), max(ys)
            pad = (ymax - ymin) * 0.2 if ymax != ymin else 0.001
            ax.set_ylim(ymin - pad, ymax + pad)

        plt.pause(0.05)
        time.sleep(0.05)

# start plotter in background
threading.Thread(target=plot_thread, daemon=True).start()

# --- BLE NOTIFICATION HANDLER ---
async def notification_handler(sender, data):
    try:
        text = data.decode("utf-8").strip()

        # Expecting format: "index:%d, %.3f"
        match = re.search(r"index:(\d+),\s*([-+]?\d*\.\d+|\d+)", text)
        if match:
            idx = int(match.group(1))
            value = float(match.group(2))
            data_buffer.append((idx, value))  # store tuple

            with open(CSV_FILENAME, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([datetime.datetime.now().isoformat(), idx, value])

        print(text)

    except Exception as e:
        print("Error decoding BLE data:", e)

# --- MAIN BLE LOOP ---
async def main():
    print("Scanning for ESP32...")
    devices = await BleakScanner.discover(timeout=5)

    if not devices:
        print("❌ No BLE devices found.")
        return

    print("Devices found:")
    for d in devices:
        print(f" - {d.name} ({d.address})")

    esp32_device = None
    for d in devices:
        if d.name and ESP32_NAME in d.name:
            esp32_device = d
            break

    if esp32_device is None:
        print(f"Could not find device named '{ESP32_NAME}'")
        return

    print(f"Found ESP32: {esp32_device.name} ({esp32_device.address})")

    async with BleakClient(esp32_device.address) as client:
        print("Connecting...")
        if not client.is_connected:
            print("Failed to connect.")
            return
        print("Connected!")

        print("\n--- Services and Characteristics ---")
        services = client.services  # Bleak fills this automatically
        for service in services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                print(f"   Char: {char.uuid}  {char.properties}")
        print("------------------------------------\n")

        # Subscribe to notifications
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        print("Listening for UTF-8 packets... Press Ctrl+C to stop.\n")

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Stopping notifications...")
            await client.stop_notify(CHARACTERISTIC_UUID)
            print("Disconnected.")

if __name__ == "__main__":
    asyncio.run(main())