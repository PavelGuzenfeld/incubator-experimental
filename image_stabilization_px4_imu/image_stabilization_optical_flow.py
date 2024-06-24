import cv2
import numpy as np
import time

# Global variables
odom_data = None
frame = None
stop_threads = False

locked_position = None
odom_samples = []
lock_start_time = None


def capture_camera():
    global frame, stop_threads, locked_position, odom_samples, lock_start_time
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("error: could not open video camera.")
            stop_threads = True
            return

        # Wait for the camera to initialize
        time.sleep(2)

        lock_start_time = time.time()

        ret, prev_frame = cap.read()
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        lk_params = dict(
            winSize=(10, 10),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        while cap.isOpened() and not stop_threads:
            ret, frame = cap.read()
            if not ret:
                print("failed to read frame from camera")
                stop_threads = True
                break

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
                    avg_motion = np.mean(motion_vectors, axis=0)

                    if locked_position is None:
                        locked_position = [0, 0]  # Initialize the locked position
                    # Use avg_motion to adjust locked position
                    locked_position[0] += avg_motion[0, 0]
                    locked_position[1] += avg_motion[0, 1]

            prev_gray = curr_gray.copy()

            if locked_position:
                draw_locked_position(frame, locked_position)
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


def draw_locked_position(frame, locked_position):
    rows, cols = frame.shape[:2]
    screen_x, screen_y = int(cols / 2 + locked_position[0]), int(
        rows / 2 + locked_position[1]
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
if __name__ == "__main__":
    capture_camera()

print("done")
