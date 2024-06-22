import cv2
import numpy as np
from pymavlink import mavutil
import time
import threading

# Constants
DATA_STREAM_RATE = 30  # hz
LOCK_DURATION = 0.2  # 200 ms
EARTH_RADIUS = 6378137.0  # in meters

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
imu_data = None
local_ned_data = None
frame = None
stop_threads = False

locked_position = None
ned_samples = []
imu_samples = []
lock_start_time = None


def get_imu_data():
    global imu_data, local_ned_data, stop_threads
    while not stop_threads:
        try:
            msg = mavlink_connection.recv_match(type="ATTITUDE", blocking=True)
            if msg:
                imu_data = (msg.roll, msg.pitch, msg.yaw)

            msg_ned = mavlink_connection.recv_match(
                type="LOCAL_POSITION_NED", blocking=True
            )
            if msg_ned:
                local_ned_data = (msg_ned.x, msg_ned.y, msg_ned.z)
        except Exception as e:
            print(f"failed to get imu or NED data: {e}")


def capture_camera():
    global frame, stop_threads, locked_position, ned_samples, imu_samples, lock_start_time
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

            if local_ned_data and imu_data:
                if locked_position is None:
                    ned_samples.append(local_ned_data)
                    imu_samples.append(imu_data)
                    if time.time() - lock_start_time >= LOCK_DURATION:
                        locked_position = calculate_locked_position(
                            ned_samples, imu_samples
                        )

                if locked_position:
                    draw_locked_position(
                        frame, locked_position, local_ned_data, imu_data
                    )
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


def calculate_locked_position(ned_samples, imu_samples):
    ned_avg = np.mean(ned_samples, axis=0)
    imu_avg = np.mean(imu_samples, axis=0)
    return project_tangent_point(ned_avg, imu_avg)


def project_tangent_point(ned_avg, imu_avg):
    roll, pitch, yaw = imu_avg
    x, y, z = ned_avg

    # Assuming these values are provided correctly
    projected_x = x / (EARTH_RADIUS + z)
    projected_y = y / (EARTH_RADIUS + z)
    projected_z = z  # Keep the same Z value

    # Return the position and the average quaternion as (w, x, y, z)
    avg_quaternion = euler_to_quaternion(roll, pitch, yaw)

    return projected_x, projected_y, projected_z, avg_quaternion


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


def draw_locked_position(frame, locked_position, local_ned_data, imu_data):
    rows, cols = frame.shape[:2]
    screen_x, screen_y = calculate_screen_coords(
        locked_position, local_ned_data, imu_data, cols, rows
    )

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


def calculate_screen_coords(
    locked_position,
    odom_data,
    imu_data,
    cols,
    rows,
    angular_weight=0,
    linear_weight=1,
):
    locked_x, locked_y, locked_z, locked_q = locked_position
    current_x, current_y, current_z = odom_data
    current_roll, current_pitch, current_yaw = imu_data
    locked_roll, locked_pitch, locked_yaw = quaternion_to_euler(locked_q)

    # Calculate relative position in NED
    rel_x = locked_x - current_x
    rel_y = locked_y - current_y
    rel_z = locked_z - current_z

    # Calculate relative angles
    delta_roll = current_roll - locked_roll
    delta_pitch = current_pitch - locked_pitch
    delta_yaw = current_yaw - locked_yaw

    # Convert angular displacements to screen coordinates
    fov_x, fov_y = 39.285, 22.70  # Half the field of view in degrees
    fov_x_rad = np.radians(fov_x)
    fov_y_rad = np.radians(fov_y)

    # Calculate the pixel displacement due to angular changes
    screen_dx = delta_yaw * cols / fov_x_rad
    screen_dy = delta_pitch * rows / fov_y_rad

    # Calculate the pixel displacement due to linear changes
    linear_dx = (rel_x / (current_z + EARTH_RADIUS)) * cols / fov_x_rad
    linear_dy = (rel_z / (current_z + EARTH_RADIUS)) * rows / fov_y_rad

    # Apply weights
    total_dx = -angular_weight * screen_dx - linear_weight * linear_dx
    total_dy = angular_weight * screen_dy + linear_weight * linear_dy

    # Center the coordinates on the screen
    screen_x = int(cols / 2 + total_dx)
    screen_y = int(rows / 2 + total_dy)

    return screen_x, screen_y


def quaternion_to_euler(q):
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw


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
imu_thread = threading.Thread(target=get_imu_data)
camera_thread = threading.Thread(target=capture_camera)

imu_thread.start()
camera_thread.start()

# Wait for threads to finish
camera_thread.join()
stop_threads = True
imu_thread.join()

print("done")
