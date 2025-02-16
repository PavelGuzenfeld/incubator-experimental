#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

int main()
{
    const char *devName = "/dev/video0";
    int fd = open(devName, O_RDWR);
    if (fd < 0)
    {
        std::cerr << "Error opening " << devName << ": " << strerror(errno) << std::endl;
        return 1;
    }

    // 1. Query device capabilities
    {
        v4l2_capability cap;
        std::memset(&cap, 0, sizeof(cap));
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0)
        {
            std::cerr << "VIDIOC_QUERYCAP error: " << strerror(errno) << std::endl;
            close(fd);
            return 1;
        }

        std::cout << "Driver: " << cap.driver << std::endl;
        std::cout << "Card: " << cap.card << std::endl;
        std::cout << "Bus: " << cap.bus_info << std::endl;
        std::cout << "Version: " << ((cap.version >> 16) & 0xFF) << "."
                  << ((cap.version >> 8) & 0xFF) << "."
                  << (cap.version & 0xFF) << std::endl;

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        {
            std::cerr << "This device does not support VIDEO_CAPTURE." << std::endl;
            close(fd);
            return 1;
        }
    }

    // 2. Enumerate supported formats (pixel formats)
    {
        std::cout << "\n=== Supported Pixel Formats ===\n";
        v4l2_fmtdesc fmtdesc;
        std::memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        while (true)
        {
            if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) < 0)
            {
                if (errno == EINVAL)
                {
                    // No more formats
                    break;
                }
                else
                {
                    std::cerr << "VIDIOC_ENUM_FMT error: " << strerror(errno) << std::endl;
                    break;
                }
            }

            std::cout << "Index: " << fmtdesc.index
                      << " FourCC: " << std::string(reinterpret_cast<char *>(&fmtdesc.pixelformat), 4)
                      << " Description: " << fmtdesc.description << std::endl;

            // 2a. For each pixel format, enumerate frame sizes
            {
                v4l2_frmsizeenum fsize;
                std::memset(&fsize, 0, sizeof(fsize));
                fsize.pixel_format = fmtdesc.pixelformat;

                std::cout << "  - Frame Sizes:\n";
                while (true)
                {
                    if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize) < 0)
                    {
                        if (errno == EINVAL)
                        {
                            // No more framesizes for this format
                            break;
                        }
                        else
                        {
                            std::cerr << "    VIDIOC_ENUM_FRAMESIZES error: " << strerror(errno) << std::endl;
                            break;
                        }
                    }

                    if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
                    {
                        std::cout << "    Discrete: "
                                  << fsize.discrete.width << "x"
                                  << fsize.discrete.height << std::endl;
                        // 2b. Enumerate frame intervals (frame rates) for each discrete size
                        {
                            v4l2_frmivalenum fival;
                            std::memset(&fival, 0, sizeof(fival));
                            fival.pixel_format = fmtdesc.pixelformat;
                            fival.width = fsize.discrete.width;
                            fival.height = fsize.discrete.height;

                            while (true)
                            {
                                if (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival) < 0)
                                {
                                    if (errno == EINVAL)
                                    {
                                        // no more intervals
                                        break;
                                    }
                                    else
                                    {
                                        std::cerr << "      VIDIOC_ENUM_FRAMEINTERVALS error: "
                                                  << strerror(errno) << std::endl;
                                        break;
                                    }
                                }

                                if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)
                                {
                                    std::cout << "      Interval: "
                                              << fival.discrete.numerator << "/"
                                              << fival.discrete.denominator << " fps\n";
                                }
                                else if (fival.type == V4L2_FRMIVAL_TYPE_CONTINUOUS)
                                {
                                    // ranges from min to max
                                    std::cout << "      Continuous: "
                                              << "(" << fival.stepwise.min.numerator << "/" << fival.stepwise.min.denominator << ") to "
                                              << "(" << fival.stepwise.max.numerator << "/" << fival.stepwise.max.denominator << ") fps\n";
                                    break; // continuous covers all
                                }
                                else if (fival.type == V4L2_FRMIVAL_TYPE_STEPWISE)
                                {
                                    std::cout << "      Stepwise: from "
                                              << fival.stepwise.min.numerator << "/" << fival.stepwise.min.denominator
                                              << " to "
                                              << fival.stepwise.max.numerator << "/" << fival.stepwise.max.denominator
                                              << " step "
                                              << fival.stepwise.step.numerator << "/"
                                              << fival.stepwise.step.denominator << "\n";
                                    break; // stepwise covers all
                                }

                                fival.index++;
                            }
                        }
                    }
                    else if (fsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
                    {
                        std::cout << "    Stepwise range: "
                                  << fsize.stepwise.min_width << "x" << fsize.stepwise.min_height
                                  << " to "
                                  << fsize.stepwise.max_width << "x" << fsize.stepwise.max_height
                                  << " (step: "
                                  << fsize.stepwise.step_width << "x" << fsize.stepwise.step_height
                                  << ")\n";
                        // For stepwise, you can pick a desired width/height in the range
                        // Then similarly enumerate intervals with VIDIOC_ENUM_FRAMEINTERVALS
                    }

                    fsize.index++;
                }
            }

            fmtdesc.index++;
        }
    }

    // 3. Enumerate controls (brightness, contrast, exposure, etc.)
    {
        std::cout << "\n=== Supported Controls ===\n";
        // Controls can live in multiple ranges. Typically:
        //   - Base IDs: [ V4L2_CID_BASE ... V4L2_CID_LASTP1-1 ]
        //   - Camera-specific IDs: [ V4L2_CID_CAMERA_CLASS_BASE ... V4L2_CID_CAMERA_CLASS_BASE+0x7ff ]
        //   - Private or driver-specific IDs beyond that

        // We'll do a generic scan of a broad range, e.g. 0x00980900 to 0x0098FFFF
        // (This is typical for camera class controls but might not cover everything.)
        for (int cid = V4L2_CID_BASE; cid < V4L2_CID_LASTP1; cid++)
        {
            struct v4l2_queryctrl qctrl;
            std::memset(&qctrl, 0, sizeof(qctrl));
            qctrl.id = cid;

            if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &qctrl))
            {
                if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED))
                {
                    std::cout << "Control: " << qctrl.name
                              << " (id=0x" << std::hex << cid << std::dec << ")\n"
                              << "  Type: " << qctrl.type << "\n"
                              << "  Min: " << qctrl.minimum << ", Max: " << qctrl.maximum
                              << ", Step: " << qctrl.step << ", Default: " << qctrl.default_value << "\n";
                }
            }
        }

        // Also check extended camera controls often at 0x009A0000..
        // (V4L2_CID_CAMERA_CLASS_BASE is 0x009A0900 in some kernels)
        for (int cid = 0x009A0000; cid < 0x009AFFFF; cid++)
        {
            struct v4l2_queryctrl qctrl;
            std::memset(&qctrl, 0, sizeof(qctrl));
            qctrl.id = cid;

            if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &qctrl))
            {
                if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED))
                {
                    std::cout << "Control: " << qctrl.name
                              << " (id=0x" << std::hex << cid << std::dec << ")\n"
                              << "  Type: " << qctrl.type << "\n"
                              << "  Min: " << qctrl.minimum << ", Max: " << qctrl.maximum
                              << ", Step: " << qctrl.step << ", Default: " << qctrl.default_value << "\n";
                }
            }
        }
    }

    // 4. Fine Tuning Based on Discovered Info
    //
    //   Once you know which pixel formats, sizes, frame rates, and controls are supported,
    //   you can pick the best settings for your application and then use:
    //
    //   - VIDIOC_S_FMT to set format, resolution, etc.
    //   - (Possibly VIDIOC_S_PARM to set streaming parameters like frame interval.)
    //   - VIDIOC_S_CTRL (or VIDIOC_S_EXT_CTRLS) to set controls (exposure, gain, etc.)
    //
    //   Then proceed with your buffer allocations (REQBUFS), streaming, etc.

    close(fd);
    return 0;
}
