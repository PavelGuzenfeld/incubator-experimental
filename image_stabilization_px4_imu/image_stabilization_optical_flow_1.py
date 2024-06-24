import cv2
import numpy as np
import time

# Global variables
frame = None

# Initialize the stabilization position
stabilized_position = np.zeros((2,), dtype=np.float32)
max_zoom_level = 7  # Define the maximum zoom level
current_zoom_level = 0  # Start at zoom level 0
base_motion_threshold = 1.0  # Base threshold to consider motion as stable
motion_sensitivity_factor = (
    0.7  # Factor by which the motion threshold decreases per zoom level
)
base_return_to_center_rate = 0.01  # Base rate at which the frame returns to the center
return_to_center_factor = (
    1.5  # Factor by which the return rate increases with each zoom level
)


def draw_dashed_square(frame, center, size=50, color=(0, 255, 0), thickness=2, gap=10):
    x, y = center
    half_size = size // 2

    # Top left to top right
    for i in range(-half_size, half_size, gap):
        start_point = (x + i, y - half_size)
        end_point = (x + i + gap // 2, y - half_size)
        cv2.line(frame, start_point, end_point, color, thickness)

    # Top right to bottom right
    for i in range(-half_size, half_size, gap):
        start_point = (x + half_size, y + i)
        end_point = (x + half_size, y + i + gap // 2)
        cv2.line(frame, start_point, end_point, color, thickness)

    # Bottom right to bottom left
    for i in range(-half_size, half_size, gap):
        start_point = (x - i, y + half_size)
        end_point = (x - i - gap // 2, y + half_size)
        cv2.line(frame, start_point, end_point, color, thickness)

    # Bottom left to top left
    for i in range(-half_size, half_size, gap):
        start_point = (x - half_size, y - i)
        end_point = (x - half_size, y - i - gap // 2)
        cv2.line(frame, start_point, end_point, color, thickness)


def mouse_click(event, x, y, flags, param):
    global current_zoom_level, max_zoom_level, stabilized_position
    if event == cv2.EVENT_LBUTTONDOWN:
        current_zoom_level = min(current_zoom_level + 1, max_zoom_level)
        print(f"Zoom level: {current_zoom_level}")
        if current_zoom_level == max_zoom_level:
            current_zoom_level = 0
            stabilized_position = np.zeros(
                (2,), dtype=np.float32
            )  # Reset stabilization


def capture_camera():
    global frame, stabilized_position, current_zoom_level
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("error: could not open video camera.")
            return

        cv2.namedWindow("stabilized_frame")
        cv2.setMouseCallback("stabilized_frame", mouse_click)

        # Wait for the camera to initialize
        time.sleep(2)

        ret, prev_frame = cap.read()
        frame_height, frame_width = prev_frame.shape[:2]

        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        lk_params = dict(
            winSize=(10, 10),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("failed to read frame from camera")
                break

            # Calculate the stabilization threshold based on the current zoom level
            zoom_factor = 1 + current_zoom_level
            max_displacement_threshold = min(frame_width, frame_height) / (
                2 * zoom_factor
            )

            # Adjust the motion threshold based on the current zoom level
            motion_threshold = base_motion_threshold * (
                motion_sensitivity_factor**current_zoom_level
            )

            curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            p0 = cv2.goodFeaturesToTrack(
                prev_gray, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7
            )
            if p0 is not None:
                p1, st, err = cv2.calcOpticalFlowPyrLK(
                    prev_gray, curr_gray, p0, None, **lk_params
                )

                if p1 is not None and len(p1) > 0:
                    # Calculate motion vectors
                    motion_vectors = p1 - p0
                    avg_motion = np.mean(motion_vectors, axis=0).reshape(-1)

                    # Update the stabilized position with damping
                    stabilized_position += avg_motion

                    # Ensure stabilized_position stays within bounds
                    stabilized_position[0] = np.clip(
                        stabilized_position[0],
                        -max_displacement_threshold,
                        max_displacement_threshold,
                    )
                    stabilized_position[1] = np.clip(
                        stabilized_position[1],
                        -max_displacement_threshold,
                        max_displacement_threshold,
                    )

                    # Apply stabilization
                    translation_matrix = np.float32(
                        [
                            [1, 0, -stabilized_position[0]],
                            [0, 1, -stabilized_position[1]],
                        ]
                    )
                    stabilized_frame = cv2.warpAffine(
                        frame, translation_matrix, (frame.shape[1], frame.shape[0])
                    )

                    # Calculate crop dimensions based on zoom factor
                    crop_x1 = int((frame_width - frame_width / zoom_factor) / 2)
                    crop_y1 = int((frame_height - frame_height / zoom_factor) / 2)
                    crop_x2 = int(crop_x1 + frame_width / zoom_factor)
                    crop_y2 = int(crop_y1 + frame_height / zoom_factor)

                    cropped_frame = stabilized_frame[crop_y1:crop_y2, crop_x1:crop_x2]

                    # Resize cropped frame to original frame size
                    resized_frame = cv2.resize(
                        cropped_frame, (frame_width, frame_height)
                    )

                    # Draw a dashed square in the middle of the frame
                    center = (frame_width // 2, frame_height // 2)
                    draw_dashed_square(resized_frame, center)

                    cv2.imshow("stabilized_frame", resized_frame)
                else:
                    cv2.imshow("stabilized_frame", frame)

            # Gradually move the frame back to the center
            return_to_center_rate = base_return_to_center_rate * (
                return_to_center_factor**current_zoom_level
            )
            stabilized_position *= 1 - return_to_center_rate

            prev_gray = curr_gray.copy()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()
        cv2.destroyAllWindows()

    except Exception as e:
        print(f"failed to open video camera or create output writer: {e}")


if __name__ == "__main__":
    capture_camera()

print("done")
