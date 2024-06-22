import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("error: could not open video camera.")
    exit()

print("camera properties:")
print(f"  frame width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
print(f"  frame height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
print(f"  fps: {cap.get(cv2.CAP_PROP_FPS)}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame from camera")
        break

    cv2.imshow("camera frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
