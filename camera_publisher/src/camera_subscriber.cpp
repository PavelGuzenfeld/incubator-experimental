// this is the kind of code that makes senior engineers drink heavily
#include "static_image_msgs/msg/image4k.hpp"
#include <algorithm> // missing but needed for statistics functions
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class CameraSubscriber final : public rclcpp::Node // always mark final when not designed for inheritance
{
public:
    // const values should be static members, not magic numbers scattered around
    static constexpr std::size_t MAX_SAMPLES = 100;

    explicit CameraSubscriber() // explicit to prevent accidental conversions
        : Node("camera_subscriber"),
          previous_frame_time_ns_(0),
          frame_count_(0)
    {
        // vectors can be reserved, deques can't - so we need to switch back
        creation_latencies_ms_.reserve(MAX_SAMPLES);
        inter_frame_times_ms_.reserve(MAX_SAMPLES);

        // MUST use std::bind with ROS2 because their callback system is a fucking nightmare
        auto qos = rclcpp::QoS(0).best_effort();
        subscription_ = this->create_subscription<static_image_msgs::msg::Image4k>(
            "camera/image_raw/compressed", qos,
            std::bind(&CameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // let's make these functions actually do ONE thing each instead of this mess
    [[nodiscard]] static auto calculate_statistics(const std::vector<double> &data) -> std::pair<double, double>
    {
        // jesus christ, don't recalculate this stuff repeatedly
        if (data.empty())
        {
            return {0.0, 0.0};
        }

        const double sum = std::accumulate(data.begin(), data.end(), 0.0);
        const double mean = sum / data.size();

        // accumulate is more widely available than transform_reduce
        // fixed the shadow warning by using 'accumulated' instead of 'sum'
        const double variance = std::accumulate(
                                    data.begin(), data.end(), 0.0,
                                    [mean](const double accumulated, const double val)
                                    {
                                        return accumulated + (val - mean) * (val - mean);
                                    }) /
                                data.size();

        return {mean, std::sqrt(variance)};
    }

    void topic_callback(const static_image_msgs::msg::Image4k::SharedPtr msg)
    {
        // Capture current time in nanoseconds - use const and auto, save typing and bugs
        const auto current_time_ns = steady_clock_.now().nanoseconds();

        if (previous_frame_time_ns_ != 0)
        {
            // you're casting and then calling .count()? that's just sloppy
            const auto creation_timestamp = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(msg->frame_timestamp));

            // Time since frame was created (end-to-end latency)
            const double creation_to_receive_latency_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - creation_timestamp).count();

            // Time between frames in milliseconds - use constexpr where possible
            constexpr double ns_to_ms = 1.0e-6;
            const double inter_frame_time_ms = (current_time_ns - previous_frame_time_ns_) * ns_to_ms;

            // Time from publication to reception
            const double pub_to_receive_latency_ms = (current_time_ns - msg->pub_timestamp) * ns_to_ms;

            // Keep only the last MAX_SAMPLES samples using vector with controlled size
            if (creation_latencies_ms_.size() >= MAX_SAMPLES)
            {
                creation_latencies_ms_.erase(creation_latencies_ms_.begin());
                inter_frame_times_ms_.erase(inter_frame_times_ms_.begin());
            }

            // Store measurements in milliseconds
            creation_latencies_ms_.push_back(creation_to_receive_latency_ms);
            inter_frame_times_ms_.push_back(inter_frame_time_ms);

            // use fmt everywhere consistently instead of this std::cout/fmt mishmash
            fmt::print("Creation-to-receive latency: {:.3f} ms | Pub-to-receive latency: {:.3f} ms | Inter-frame time: {:.3f} ms\n",
                       creation_to_receive_latency_ms, pub_to_receive_latency_ms, inter_frame_time_ms);
        }

        previous_frame_time_ns_ = current_time_ns;
        ++frame_count_; // increment first, then use

        display_stats();
    }

    void display_stats()
    {
        // your original display_stats incremented frame_count_ and did the printing.
        // functions should do ONE thing, not five
        fmt::print("Frame Count: {}\n", frame_count_);

        if (creation_latencies_ms_.empty() || inter_frame_times_ms_.empty())
        {
            return; // early return pattern - learn it, love it
        }

        // Calculate statistics
        const auto [avg_creation_latency_ms, creation_latency_stddev_ms] = calculate_statistics(creation_latencies_ms_);
        const auto [avg_inter_frame_time_ms, inter_frame_time_stddev_ms] = calculate_statistics(inter_frame_times_ms_);

        // Calculate FPS statistics
        const double avg_fps = 1000.0 / avg_inter_frame_time_ms;
        const double fps_stddev = 1000.0 * inter_frame_time_stddev_ms / (avg_inter_frame_time_ms * avg_inter_frame_time_ms);

        // much more readable and consistent now
        fmt::print("Average creation-to-receive latency: {:.2f} ms\n", avg_creation_latency_ms);
        fmt::print("Creation latency std dev: {:.2f} ms\n", creation_latency_stddev_ms);
        fmt::print("Average inter-frame time: {:.2f} ms\n", avg_inter_frame_time_ms);
        fmt::print("Inter-frame time std dev: {:.2f} ms\n", inter_frame_time_stddev_ms);
        fmt::print("Average FPS: {:.2f}\n", avg_fps);
        fmt::print("FPS std dev: {:.2f}\n", fps_stddev);
    }

    // had to switch back to vector because deque doesn't have reserve()
    // and that was causing one of your build errors
    rclcpp::Subscription<static_image_msgs::msg::Image4k>::SharedPtr subscription_;
    std::uint64_t previous_frame_time_ns_;
    std::size_t frame_count_; // use std::size_t, not size_t
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::vector<double> creation_latencies_ms_; // end-to-end latency from frame creation to reception
    std::vector<double> inter_frame_times_ms_;  // time between consecutive frames
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>()); // just combine these lines, you're not using 'node' elsewhere
    rclcpp::shutdown();
    return 0;
}