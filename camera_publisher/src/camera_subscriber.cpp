#include "static_image_msgs/msg/image4k.hpp"
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
        : Node("camera_subscriber"),
          previous_ros_time_(0, 0, RCL_STEADY_TIME), frame_count_(0)
    {
        auto qos = rclcpp::QoS(0).best_effort();
        subscription_ = this->create_subscription<static_image_msgs::msg::Image4k>(
            "camera/image_raw/compressed", qos,
            std::bind(&CameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const static_image_msgs::msg::Image4k::SharedPtr msg)
    {
        // grab "now" from the steady clock
        auto now = steady_clock_.now();

        // frame timestamps are already steady time domain
        rclcpp::Time frame_creation_time(msg->frame_timestamp, RCL_STEADY_TIME);
        rclcpp::Time frame_pub_time(msg->pub_timestamp, RCL_STEADY_TIME);

        if (previous_ros_time_ != rclcpp::Time(0, 0, RCL_STEADY_TIME))
        {
            auto latency_creation = now - frame_creation_time;
            auto latency_pub = now - frame_pub_time;

            double latency_creation_ms = latency_creation.seconds() * 1000.0;
            double latency_pub_ms = latency_pub.seconds() * 1000.0;

            latencies_.push_back(latency_creation_ms);
            frame_times_.push_back((now - previous_ros_time_).seconds() * 1000.0);

            if (latencies_.size() > 100)
                latencies_.erase(latencies_.begin());
            if (frame_times_.size() > 100)
                frame_times_.erase(frame_times_.begin());

            fmt::print("Latency since creation: {:.3f} ms | Latency since pub: {:.3f} ms\n",
                       latency_creation_ms, latency_pub_ms);
        }

        previous_ros_time_ = now;

        display_stats();
    }

    void display_stats()
    {
        frame_count_++;
        std::cout << "Frame Count: " << frame_count_ << "\n";

        if (!latencies_.empty() && !frame_times_.empty())
        {
            double avg_latency = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
            double avg_fps = 1000.0 / (std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) / frame_times_.size());

            double latency_stddev = std::sqrt(std::accumulate(latencies_.begin(), latencies_.end(), 0.0, [avg_latency](double sum, double val)
                                                              { return sum + (val - avg_latency) * (val - avg_latency); }) /
                                              latencies_.size());

            double fps_stddev = std::sqrt(std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0, [avg_fps](double sum, double val)
                                                          {
                double fps = 1000.0 / val;
                return sum + (fps - avg_fps) * (fps - avg_fps); }) /
                                          frame_times_.size());

            std::cout << "Average Latency: " << avg_latency << " ms\n";
            std::cout << "Latency Std Dev: " << latency_stddev << " ms\n";
            std::cout << "Average FPS: " << avg_fps << "\n";
            std::cout << "FPS Std Dev: " << fps_stddev << "\n";
        }
    }

    rclcpp::Subscription<static_image_msgs::msg::Image4k>::SharedPtr subscription_;
    rclcpp::Time previous_ros_time_;
    size_t frame_count_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::vector<double> latencies_;
    std::vector<double> frame_times_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
