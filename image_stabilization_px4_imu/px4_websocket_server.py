import asyncio
import json
import sys
import time
from pymavlink import mavutil
import websockets
import math

# Constants
FOV_X = math.radians(60)  # Horizontal FOV in radians
FOV_Y = math.radians(45)  # Vertical FOV in radians
CANVAS_WIDTH = 800  # Canvas width in pixels
CANVAS_HEIGHT = 600  # Canvas height in pixels
CALIBRATION_TIME = 1.0  # Time for calibration in seconds
DATA_STREAM_RATE = 50  # Rate of data stream in Hz
MOVEMENT_THRESHOLD = 6.0  # Threshold for ignoring small movements in pixels

def normalize_angle_difference(angle):
    """Normalize the angle to be within the range -pi to pi."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def calculate_deltas(rest_attitude, curr_attitude, canvas_width, canvas_height):
    """Calculate the deltas from the rest position in terms of pitch and yaw."""
    normalized_pitch = normalize_angle_difference(curr_attitude['pitch'] - rest_attitude['pitch'])
    normalized_yaw = normalize_angle_difference(curr_attitude['yaw'] - rest_attitude['yaw'])

    # Map the angles to pixel values on the canvas
    dx = (normalized_yaw / FOV_X) * canvas_width
    dy = (normalized_pitch / FOV_Y) * canvas_height

    return -dx, dy

def apply_movement_threshold(last_deltas, current_deltas, threshold):
    """Apply a threshold to ignore small changes from the last measurement."""
    last_dx, last_dy = last_deltas
    current_dx, current_dy = current_deltas
    if abs(current_dx - last_dx) < threshold:
        current_dx = last_dx
    if abs(current_dy - last_dy) < threshold:
        current_dy = last_dy
    return current_dx, current_dy

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
    last_deltas = (0, 0)

    while True:
        msg = mavlink_connection.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            curr_attitude = {'roll': msg.roll, 'pitch': msg.pitch, 'yaw': msg.yaw}
            dx, dy = calculate_deltas(rest_attitude, curr_attitude, CANVAS_WIDTH, CANVAS_HEIGHT)
            dx, dy = apply_movement_threshold(last_deltas, (dx, dy), MOVEMENT_THRESHOLD)
            last_deltas = (dx, dy)
            deltas = {'dx': dx, 'dy': dy}
            print(f"Sending delta data: {deltas}")
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

if __name__ == "__main__":
    asyncio.run(main())
