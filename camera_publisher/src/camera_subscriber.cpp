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
          previous_frame_time_ns_(0), frame_count_(0)
    {
        auto qos = rclcpp::QoS(0).best_effort();
        subscription_ = this->create_subscription<static_image_msgs::msg::Image4k>(
            "camera/image_raw/compressed", qos,
            std::bind(&CameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const static_image_msgs::msg::Image4k::SharedPtr msg)
    {
        // Capture current time in nanoseconds
        auto current_time_ns = steady_clock_.now().nanoseconds();

        if (previous_frame_time_ns_ != 0)
        {
            // Time since frame was created (end-to-end latency)
            double creation_to_receive_latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                        std::chrono::steady_clock::now() -
                                                        std::chrono::steady_clock::time_point(
                                                            std::chrono::nanoseconds(msg->frame_timestamp)))
                                                        .count();

            // Time between frames in milliseconds
            double inter_frame_time_ms = (current_time_ns - previous_frame_time_ns_) / 1.0e6;

            // Time from publication to reception
            double pub_to_receive_latency_ms = (current_time_ns - msg->pub_timestamp) / 1.0e6;

            // Store measurements in milliseconds
            creation_latencies_ms_.push_back(creation_to_receive_latency_ms);
            inter_frame_times_ms_.push_back(inter_frame_time_ms);

            // Keep only last 100 samples
            if (creation_latencies_ms_.size() > 100)
                creation_latencies_ms_.erase(creation_latencies_ms_.begin());
            if (inter_frame_times_ms_.size() > 100)
                inter_frame_times_ms_.erase(inter_frame_times_ms_.begin());

            fmt::print("Creation-to-receive latency: {:.3f} ms | Pub-to-receive latency: {:.3f} ms | Inter-frame time: {:.3f} ms\n",
                       creation_to_receive_latency_ms, pub_to_receive_latency_ms, inter_frame_time_ms);
        }

        previous_frame_time_ns_ = current_time_ns;

        display_stats();
    }

    void display_stats()
    {
        frame_count_++;
        std::cout << "Frame Count: " << frame_count_ << "\n";

        if (!creation_latencies_ms_.empty() && !inter_frame_times_ms_.empty())
        {
            // Calculate averages
            double avg_creation_latency_ms = std::accumulate(creation_latencies_ms_.begin(), creation_latencies_ms_.end(), 0.0) /
                                             creation_latencies_ms_.size();
            double avg_inter_frame_time_ms = std::accumulate(inter_frame_times_ms_.begin(), inter_frame_times_ms_.end(), 0.0) /
                                             inter_frame_times_ms_.size();
            double avg_fps = 1000.0 / avg_inter_frame_time_ms;

            // Calculate standard deviations
            double creation_latency_stddev_ms = std::sqrt(std::accumulate(creation_latencies_ms_.begin(), creation_latencies_ms_.end(), 0.0,
                                                                          [avg_creation_latency_ms](double sum, double val)
                                                                          {
                                                                              return sum + (val - avg_creation_latency_ms) * (val - avg_creation_latency_ms);
                                                                          }) /
                                                          creation_latencies_ms_.size());

            double inter_frame_time_stddev_ms = std::sqrt(std::accumulate(inter_frame_times_ms_.begin(), inter_frame_times_ms_.end(), 0.0,
                                                                          [avg_inter_frame_time_ms](double sum, double val)
                                                                          {
                                                                              return sum + (val - avg_inter_frame_time_ms) * (val - avg_inter_frame_time_ms);
                                                                          }) /
                                                          inter_frame_times_ms_.size());

            // FPS standard deviation using error propagation formula
            double fps_stddev = 1000.0 * inter_frame_time_stddev_ms / (avg_inter_frame_time_ms * avg_inter_frame_time_ms);

            // Print statistics with clear labels
            std::cout << "Average creation-to-receive latency: " << avg_creation_latency_ms << " ms\n";
            std::cout << "Creation latency std dev: " << creation_latency_stddev_ms << " ms\n";
            std::cout << "Average inter-frame time: " << avg_inter_frame_time_ms << " ms\n";
            std::cout << "Inter-frame time std dev: " << inter_frame_time_stddev_ms << " ms\n";
            std::cout << "Average FPS: " << avg_fps << "\n";
            std::cout << "FPS std dev: " << fps_stddev << "\n";
        }
    }

    rclcpp::Subscription<static_image_msgs::msg::Image4k>::SharedPtr subscription_;
    std::uint64_t previous_frame_time_ns_;
    size_t frame_count_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::vector<double> creation_latencies_ms_; // end-to-end latency from frame creation to reception
    std::vector<double> inter_frame_times_ms_;  // time between consecutive frames
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}