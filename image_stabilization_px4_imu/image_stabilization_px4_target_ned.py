import cv2
import numpy as np
from pymavlink import mavutil
import time
import threading

# Constants
DATA_STREAM_RATE = 30  # hz
LOCK_DURATION = 0.2  # 200 ms
EARTH_RADIUS = 10 # in meters
MIN_DISTANCE_TO_TARGET = 1.0  # Minimum distance to target in meters
PITCH_DOWN_ANGLE = np.radians(15)  # 15 degrees pitch down

# Connect to the drone
try:
    mavlink_connection = mavutil.mavlink_connection("com4")
    print("connected to mavlink")

    # Request data stream
    mavlink_connection.mav.request_data_stream_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        DATA_STREAM_RATE,
        1,
    )
    print("requested data stream")
except Exception as e:
    print(f"failed to connect to mavlink: {e}")
    exit()

# Global variables
odom_data = None
frame = None
stop_threads = False

locked_position = None
odom_samples = []
imu_samples = []
lock_start_time = None


def get_odometry_data():
    global odom_data, stop_threads
    while not stop_threads:
        try:
            msg = mavlink_connection.recv_match(type="ODOMETRY", blocking=True)
            if msg:
                odom_data = {
                    "x": msg.x,
                    "y": msg.y,
                    "z": msg.z,
                    "q": (msg.q[0], msg.q[1], msg.q[2], msg.q[3]),
                    "vx": msg.vx,
                    "vy": msg.vy,
                    "vz": msg.vz,
                    "rollspeed": msg.rollspeed,
                    "pitchspeed": msg.pitchspeed,
                    "yawspeed": msg.yawspeed,
                    "pose_covariance": msg.pose_covariance,
                    "frame_id": msg.frame_id,
                    "child_frame_id": msg.child_frame_id,
                }
        except Exception as e:
            print(f"failed to get odometry data: {e}")


def capture_camera():
    global frame, stop_threads, locked_position, odom_samples, lock_start_time
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("error: could not open video camera.")
            stop_threads = True
            return

        print_camera_properties(cap)

        # Wait for the camera to initialize
        time.sleep(2)

        lock_start_time = time.time()

        while cap.isOpened() and not stop_threads:
            ret, frame = cap.read()
            if not ret:
                print("failed to read frame from camera")
                stop_threads = True
                break

            if odom_data:
                if locked_position is None:
                    odom_samples.append(odom_data)
                    if time.time() - lock_start_time >= LOCK_DURATION:
                        locked_position = calculate_locked_position(odom_samples)

                if locked_position:
                    draw_locked_position(frame, locked_position, odom_data)
                    cv2.imshow("frame", frame)
                else:
                    cv2.imshow("frame", frame)
            else:
                cv2.imshow("frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_threads = True
                break

        cap.release()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"failed to open video camera or create output writer: {e}")
        stop_threads = True


def calculate_locked_position(odom_samples):
    odom_avg = {
        "x": np.mean([d["x"] for d in odom_samples]),
        "y": np.mean([d["y"] for d in odom_samples]),
        "z": np.mean([d["z"] for d in odom_samples]),
        "q": np.mean([d["q"] for d in odom_samples], axis=0),
    }
    # # Calculate the target NED position based on a minimal down height and the attitude
    # q = odom_avg["q"]
    # roll, pitch, yaw = quaternion_to_euler(q)

    # # Down (z) component based on minimum distance to target
    # ned_z = -MIN_DISTANCE_TO_TARGET

    # # North (x) and East (y) components based on the attitude and minimal distance
    # ned_x = MIN_DISTANCE_TO_TARGET * np.cos(pitch) * np.cos(yaw)
    # ned_y = MIN_DISTANCE_TO_TARGET * np.cos(pitch) * np.sin(yaw)
    ned_z = 0
    ned_x = 0
    ned_y = 0

    return (
        odom_avg["x"] + ned_x,
        odom_avg["y"] + ned_y,
        odom_avg["z"] + ned_z,
        odom_avg["q"],
    )


def euler_to_quaternion(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


def quaternion_to_euler(q):
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, +1.0)  # more concise and avoids the idiotic repetition
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw


def projection_on_perpendicular_to_yaw(north, east, yaw):
    # perpendicular to yaw vector
    perp_yaw_vector = np.array([-np.sin(yaw), np.cos(yaw)])

    # ned vector
    ned_vector = np.array([north, east])

    # projection onto perpendicular vector
    projection = np.dot(ned_vector, perp_yaw_vector)

    return projection


def calculate_screen_coords(locked_position, odom_data, cols, rows):
    locked_x, locked_y, locked_z, locked_q = locked_position
    current_x, current_y, current_z = odom_data["x"], odom_data["y"], odom_data["z"]
    current_roll, current_pitch, current_yaw = quaternion_to_euler(odom_data["q"])
    locked_roll, locked_pitch, locked_yaw = quaternion_to_euler(locked_q)

    # Calculate relative position in NED
    rel_x = locked_x - current_x  # North
    rel_y = locked_y - current_y  # East
    rel_z = locked_z - current_z  # Down

    # Handle division by zero
    if current_z == 0:
        current_z = 1e-6  # Small value to avoid division by zero

    # Convert NED displacement to pixel displacement
    fov_x, fov_y = 39.285, 22.70  # Camera FOV in degrees, already half FOV
    fov_x_rad = np.radians(fov_x)
    fov_y_rad = np.radians(fov_y)
    
    north_east_projection = projection_on_perpendicular_to_yaw(rel_x, rel_y, current_yaw)

    screen_dx = (
        north_east_projection
        * cols
        / (2 * np.tan(fov_x_rad) * (current_z))
    )
    screen_dy = rel_z * rows / (2 * np.tan(fov_y_rad) * (current_z))

    print(f"screen_dx: {screen_dx}, screen_dy: {screen_dy}")

    # Center the coordinates on the screen
    screen_x = int(cols / 2 + screen_dx)
    screen_y = int(rows / 2 - screen_dy)

    return screen_x, screen_y


def draw_locked_position(frame, locked_position, odom_data):
    rows, cols = frame.shape[:2]
    screen_x, screen_y = calculate_screen_coords(locked_position, odom_data, cols, rows)

    # Draw green dashed rectangle
    color = (0, 255, 0)  # Green color
    thickness = 2
    rect_width, rect_height = 50, 50  # Adjust as needed

    cv2.rectangle(
        frame,
        (screen_x - rect_width // 2, screen_y - rect_height // 2),
        (screen_x + rect_width // 2, screen_y + rect_height // 2),
        color,
        thickness,
        lineType=cv2.LINE_4,
    )


def print_camera_properties(cap):
    camera_properties = {
        "frame width": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        "frame height": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "fps": cap.get(cv2.CAP_PROP_FPS),
        "format": cap.get(cv2.CAP_PROP_FORMAT),
        "mode": cap.get(cv2.CAP_PROP_MODE),
        "brightness": cap.get(cv2.CAP_PROP_BRIGHTNESS),
        "contrast": cap.get(cv2.CAP_PROP_CONTRAST),
        "saturation": cap.get(cv2.CAP_PROP_SATURATION),
        "hue": cap.get(cv2.CAP_PROP_HUE),
        "gain": cap.get(cv2.CAP_PROP_GAIN),
        "exposure": cap.get(cv2.CAP_PROP_EXPOSURE),
        "white balance": cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
        "rectification": cap.get(cv2.CAP_PROP_RECTIFICATION),
    }

    print("camera properties:")
    for prop, value in camera_properties.items():
        print(f"  {prop}: {value}")


# Start threads
odom_thread = threading.Thread(target=get_odometry_data)
camera_thread = threading.Thread(target=capture_camera)

odom_thread.start()
camera_thread.start()

# Wait for threads to finish
camera_thread.join()
stop_threads = True
camera_thread.join()

print("done")
