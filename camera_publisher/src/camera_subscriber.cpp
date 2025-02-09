#include <rclcpp/rclcpp.hpp>
#include "static_image_msgs/msg/image4k.hpp"
#include <vector>
#include <numeric>
#include <cmath>
#include <chrono>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
    : Node("camera_subscriber"), frame_count_(0)
    {
        auto qos = rclcpp::QoS(0).best_effort();
        subscription_ = this->create_subscription<static_image_msgs::msg::Image4k>(
            "camera/image_raw/compressed", qos,
            std::bind(&CameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const static_image_msgs::msg::Image4k::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();
        auto msg_time = rclcpp::Time(msg->timestamp);

        if (previous_steady_time_ != std::chrono::steady_clock::time_point())
        {
            auto delta_ns = now - previous_steady_time_;
            double latency = std::chrono::duration_cast<std::chrono::nanoseconds>(delta_ns).count() / 1e6; // ms

            latencies_.push_back(latency);
            frame_times_.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(delta_ns).count());
            if (latencies_.size() > 100) latencies_.erase(latencies_.begin());
            if (frame_times_.size() > 100) frame_times_.erase(frame_times_.begin());
        }

        previous_steady_time_ = now;

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

            double latency_stddev = std::sqrt(std::accumulate(latencies_.begin(), latencies_.end(), 0.0, [avg_latency](double sum, double val) {
                return sum + (val - avg_latency) * (val - avg_latency);
            }) / latencies_.size());

            double fps_stddev = std::sqrt(std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0, [avg_fps](double sum, double val) {
                double fps = 1000.0 / val;
                return sum + (fps - avg_fps) * (fps - avg_fps);
            }) / frame_times_.size());

            std::cout << "Average Latency: " << avg_latency << " ms\n";
            std::cout << "Latency Std Dev: " << latency_stddev << " ms\n";
            std::cout << "Average FPS: " << avg_fps << "\n";
            std::cout << "FPS Std Dev: " << fps_stddev << "\n";
        }
    }

    rclcpp::Subscription<static_image_msgs::msg::Image4k>::SharedPtr subscription_;
    std::chrono::steady_clock::time_point previous_steady_time_;
    size_t frame_count_;
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
