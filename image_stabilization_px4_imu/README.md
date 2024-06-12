
# Image Stabilization with PX4 IMU

```bash
sudo gst-launch-1.0 -v v4l2src device=/dev/video0 ! image/jpeg, width=1280, height=720, framerate=30/1 ! tcpserversink host=192.168.1.238 port=8765
```
