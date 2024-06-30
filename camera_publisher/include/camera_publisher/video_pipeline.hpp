#pragma once
#include <string_view>
#include <span>
#include <cstdint>
#include <functional>

using FrameCallback =  std::function<void(uint64_t timestamp, size_t frame_size, std::string_view frame_format, std::span<unsigned char> frame)>;

class VideoPipeline
{
public:
    VideoPipeline(std::string_view pipeline_description, FrameCallback frame_callback);
    ~VideoPipeline();
    VideoPipeline(const VideoPipeline&) = delete;
    VideoPipeline& operator=(const VideoPipeline&) = delete;
    VideoPipeline(VideoPipeline&&) = delete;
    VideoPipeline& operator=(VideoPipeline&&) = delete;

    bool start();
    void stop();
};

