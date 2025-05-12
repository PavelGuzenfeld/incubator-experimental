# This message contains a 4K image (3840x2160) in I420 format.

builtin_interfaces/Time frame_timestamp # Monotonic time when the image was captured
builtin_interfaces/Time pub_timestamp # Monotonic time when the image was published

uint8[12441600] data #[3840*2160*3/2]

uint32 HEIGHT=2160
uint32 WIDTH=3840
string ENCODING="i420"
