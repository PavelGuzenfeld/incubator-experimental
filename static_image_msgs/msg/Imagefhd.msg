# This message contains a full HD image (1920x1080) in I420 format.

uint64 frame_timestamp # Monotonic time when the image was captured
uint64 pub_timestamp # Monotonic time when the image was published

uint8[3110400] data #[1920*1080*3/2] data

uint32 HEIGHT=1080
uint32 WIDTH=1920
string ENCODING="i420"
