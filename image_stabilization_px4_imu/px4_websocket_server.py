import asyncio
import json
import sys
import time
from pymavlink import mavutil
import websockets
import math
from collections import deque
import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Constants
FOV_X = math.radians(39.285 *2)  # Horizontal FOV in radians
FOV_Y = math.radians(22.70 *2)  # Vertical FOV in radians
CANVAS_WIDTH = 800  # Canvas width in pixels
CANVAS_HEIGHT = 600  # Canvas height in pixels
CALIBRATION_TIME = 1.0  # Time for calibration in seconds
DATA_STREAM_RATE = 30  # Rate of data stream in Hz
MOVEMENT_THRESHOLD = 0.005  # Threshold for ignoring small movements in rad/sec
FILTER_WINDOW_SIZE = 0.4  # Size of the moving average filter window in seconds

# Global variables
last_attitude = {'roll': 0, 'pitch': 0, 'yaw': 0, 'time_boot_ms': 0, 'rollspeed': 0, 'pitchspeed': 0, 'yawspeed': 0}
data = {'timestamp': [], 'dx': [], 'dy': [], 'rollspeed_deg_per_sec': [], 'pitchspeed_deg_per_sec': [], 'yawspeed_deg_per_sec': []}

def normalize_angle_difference(angle):
    """Normalize the angle to be within the range -pi to pi."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def calculate_deltas(rest_attitude, curr_attitude, canvas_width, canvas_height, last_dx, apply_filter=True):
    """Calculate the deltas from the rest position in terms of pitch and yaw."""
    normalized_pitch = normalize_angle_difference(curr_attitude['pitch'] - rest_attitude['pitch'])
    normalized_yaw = normalize_angle_difference(curr_attitude['yaw'] - rest_attitude['yaw'])

    # Map the angles to pixel values on the canvas
    dx = (normalized_yaw / FOV_X) * canvas_width * apply_filter - (1-apply_filter) * last_dx
    dy = (normalized_pitch / FOV_Y) * canvas_height

    return -dx, dy

def apply_speed_filter(d_yawspeed, threshold):
    """Apply a filter based on angular speeds to ignore small changes."""
    return abs(d_yawspeed) > threshold


def moving_average_filter(values, window_size):
    """Apply a moving average filter to the list of values."""
    return sum(values) / len(values)

async def px4_data(websocket, path):
    """Handle PX4 data and send it via WebSocket."""
    print("Waiting for heartbeat...")
    mavlink_connection.wait_heartbeat()
    print("Heartbeat received. Starting data stream...")

    # Request attitude data stream from PX4
    mavlink_connection.mav.request_data_stream_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        DATA_STREAM_RATE, 1
    )

    # Collect data for calibration
    rest_attitude = await calibrate_rest_position()
    window_size = int(FILTER_WINDOW_SIZE * DATA_STREAM_RATE)
    last_dx = 0 # Initialize last_dx to 0
    # Open CSV file for writing
    with open('px4_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'dx', 'dy', 'rollspeed_deg_per_sec', 'pitchspeed_deg_per_sec', 'yawspeed_deg_per_sec'])

        while True:
            msg = mavlink_connection.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                curr_attitude = {
                    'roll': msg.roll,
                    'pitch': msg.pitch,
                    'yaw': msg.yaw,
                    'time_boot_ms': msg.time_boot_ms,
                    'rollspeed': msg.rollspeed,
                    'pitchspeed': msg.pitchspeed,
                    'yawspeed': msg.yawspeed
                }
                
                d_yawspeed = curr_attitude['yawspeed'] - last_attitude['yawspeed']

                yaw_speed_filter_result = apply_speed_filter(d_yawspeed, MOVEMENT_THRESHOLD)
                
                dx, dy = calculate_deltas(rest_attitude,curr_attitude, CANVAS_WIDTH, CANVAS_HEIGHT, last_dx, yaw_speed_filter_result)
                last_dx = dx

                # Calculate angular speeds in degrees per second
                rollspeed_deg_per_sec = curr_attitude['rollspeed'] * 180 / math.pi
                pitchspeed_deg_per_sec = curr_attitude['pitchspeed'] * 180 / math.pi
                yawspeed_deg_per_sec = curr_attitude['yawspeed'] * 180 / math.pi

                # Write data to CSV
                writer.writerow([msg.time_boot_ms, dx, dy, rollspeed_deg_per_sec, pitchspeed_deg_per_sec, yawspeed_deg_per_sec])

                # Append data to the in-memory dictionary for plotting
                data['timestamp'].append(msg.time_boot_ms)
                data['dx'].append(dx)
                data['dy'].append(dy)
                data['rollspeed_deg_per_sec'].append(rollspeed_deg_per_sec)
                data['pitchspeed_deg_per_sec'].append(pitchspeed_deg_per_sec)
                data['yawspeed_deg_per_sec'].append(yawspeed_deg_per_sec)
                
                deltas = {'dx': dx, 'dy':dy}
                last_msg = msg
                await websocket.send(json.dumps(deltas))
                
            await asyncio.sleep(0.00001)

async def calibrate_rest_position():
    """Calibrate the rest position based on initial attitude readings."""
    calibration_data = []
    start_time = time.time()

    while time.time() - start_time < CALIBRATION_TIME:
        msg = mavlink_connection.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            attitude = {'roll': msg.roll, 'pitch': msg.pitch, 'yaw': msg.yaw}
            calibration_data.append(attitude)

    rest_attitude = {
        'roll': sum(d['roll'] for d in calibration_data) / len(calibration_data),
        'pitch': sum(d['pitch'] for d in calibration_data) / len(calibration_data),
        'yaw': sum(d['yaw'] for d in calibration_data) / len(calibration_data),
    }

    print(f"Calibrated rest position: {rest_attitude}")
    return rest_attitude

async def main():
    """Main function to start the WebSocket server."""
    if len(sys.argv) != 2:
        print("Usage: python px4_websocket_server.py <COM_PORT>")
        sys.exit(1)

    serial_port = sys.argv[1]
    print(f"Connecting to PX4 on {serial_port}...")
    global mavlink_connection
    mavlink_connection = mavutil.mavlink_connection(serial_port)

    async with websockets.serve(px4_data, 'localhost', 8765):
        print("Starting WebSocket server on ws://localhost:8765")
        await asyncio.Future()  # run forever

def animate(i):
    """Update the graph with new data."""
    plt.clf()

    ax1 = plt.subplot(4, 1, 1)
    ax1.plot(data['timestamp'], data['dx'], label='dx')
    ax1.set_xlabel('Timestamp (ms)')
    ax1.set_ylabel('dx')
    ax1.legend()

    ax2 = plt.subplot(4, 1, 2)
    ax2.plot(data['timestamp'], data['dy'], label='dy', color='orange')
    ax2.set_xlabel('Timestamp (ms)')
    ax2.set_ylabel('dy')
    ax2.legend()

    ax3 = plt.subplot(4, 1, 3)
    ax3.plot(data['timestamp'], data['yawspeed_deg_per_sec'], label='Yaw Speed (deg/sec)', color='green')
    ax3.set_xlabel('Timestamp (ms)')
    ax3.set_ylabel('Yaw Speed (deg/sec)')
    ax3.legend()
    
    ax4 = plt.subplot(4, 1, 4)
    ax4.plot(data['timestamp'], data['pitchspeed_deg_per_sec'], label='Pitch Speed (deg/sec)', color='purple')
    ax4.set_xlabel('Timestamp (ms)')
    ax4.set_ylabel('Pitch Speed (deg/sec)')
    ax4.legend()

    plt.tight_layout()

def run_websocket_server():
    asyncio.run(main())

if __name__ == "__main__":
    # Start the WebSocket server in a separate thread
    threading.Thread(target=run_websocket_server, daemon=True).start()

    # Start the plot animation
    fig = plt.figure(figsize=(12, 8))
    ani = FuncAnimation(fig, animate, interval=100)
    plt.show()
