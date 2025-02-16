#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

// Some older kernel headers may not define these:
#ifndef V4L2_CID_TIMESTAMP_SOURCE
#define V4L2_CID_TIMESTAMP_SOURCE (V4L2_CID_USER_BASE + 0x1029)
#endif

#ifndef V4L2_TIMESTAMP_SRC_EOF
#define V4L2_TIMESTAMP_SRC_EOF 0
#define V4L2_TIMESTAMP_SRC_SOE 1
#endif

struct Buffer
{
    void *start;
    size_t length;
};

int main()
{
    // 0. Open V4L2 device
    int fd = open("/dev/video0", O_RDWR);
    if (fd < 0)
    {
        perror("Cannot open /dev/video0");
        return 1;
    }

    // 1. Attempt to set timestamp source to Start-Of-Exposure (SOE), if supported
    {
        struct v4l2_queryctrl qctrl;
        std::memset(&qctrl, 0, sizeof(qctrl));
        qctrl.id = V4L2_CID_TIMESTAMP_SOURCE;

        if (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0)
        {
            struct v4l2_control ctrl;
            std::memset(&ctrl, 0, sizeof(ctrl));
            ctrl.id = V4L2_CID_TIMESTAMP_SOURCE;
            ctrl.value = V4L2_TIMESTAMP_SRC_SOE;

            if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0)
            {
                std::cout << "[INFO] Successfully set timestamp source to START-OF-EXPOSURE.\n";
            }
            else
            {
                perror("[WARN] VIDIOC_S_CTRL (SOE) failed");
            }
        }
        else
        {
            std::cout << "[WARN] Driver does not support V4L2_CID_TIMESTAMP_SOURCE.\n"
                      << "       It may still provide real-time or monotonic EOF timestamps.\n";
        }
    }

    // 2. Set capture format
    struct v4l2_format fmt;
    std::memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // or any format your camera supports
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    {
        perror("VIDIOC_S_FMT");
        close(fd);
        return 1;
    }

    // 3. Request MMAP buffers
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0)
    {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return 1;
    }

    Buffer *buffers = static_cast<Buffer *>(std::calloc(req.count, sizeof(Buffer)));
    if (!buffers)
    {
        std::cerr << "Failed to allocate buffer array\n";
        close(fd);
        return 1;
    }

    // 4. Query & mmap each buffer
    for (unsigned int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            perror("VIDIOC_QUERYBUF");
            free(buffers);
            close(fd);
            return 1;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(nullptr, buf.length,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED)
        {
            perror("mmap");
            free(buffers);
            close(fd);
            return 1;
        }
    }

    // 5. Queue the buffers
    for (unsigned int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
        {
            perror("VIDIOC_QBUF");
            // Cleanup
            for (unsigned int j = 0; j <= i; j++)
            {
                munmap(buffers[j].start, buffers[j].length);
            }
            free(buffers);
            close(fd);
            return 1;
        }
    }

    // 6. Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        perror("VIDIOC_STREAMON");
        // Cleanup
        for (unsigned int i = 0; i < req.count; i++)
        {
            munmap(buffers[i].start, buffers[i].length);
        }
        free(buffers);
        close(fd);
        return 1;
    }

    // 7. Capture some frames
    for (unsigned int frame = 0; frame < 10; frame++)
    {
        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Dequeue
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0)
        {
            perror("VIDIOC_DQBUF");
            break;
        }

        // Check if the driver claims SOE in this buffer's flags
        bool isSOE = (buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_SOE);

        // Driver timestamp
        time_t sec = buf.timestamp.tv_sec;
        suseconds_t usec = buf.timestamp.tv_usec;
        double driverTimestamp = static_cast<double>(sec) + static_cast<double>(usec) / 1e6;

        // Get current system MONOTONIC time
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double nowSeconds = static_cast<double>(now.tv_sec) +
                            static_cast<double>(now.tv_nsec) / 1e9;

        // Calculate latency (now - driverTimestamp)
        // This is only meaningful if driverTimestamp is also monotonic.
        double latency = nowSeconds - driverTimestamp; // in seconds
        double latencyMs = latency * 1000.0;

        std::cout << "Frame " << frame
                  << " captured at driverTimestamp=" << driverTimestamp << "s"
                  << (isSOE ? " [START-OF-EXPOSURE]" : "")
                  << " => systemNow=" << nowSeconds << "s"
                  << ", latency=" << latencyMs << " ms"
                  << std::endl;

        // (Optionally process buffers[buf.index].start ...)

        // Re-queue
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0)
        {
            perror("VIDIOC_QBUF");
            break;
        }
    }

    // 8. Stop streaming
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        perror("VIDIOC_STREAMOFF");
    }

    // 9. Cleanup
    for (unsigned int i = 0; i < req.count; i++)
    {
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    close(fd);

    return 0;
}
