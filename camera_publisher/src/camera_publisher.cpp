#include "video_pipeline.hpp"
#include <rclcpp/rclcpp.hpp>
#include "static_image_msgs/msg/image4k.hpp"
#include <chrono>
#include <vector>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() 
    : Node("camera_publisher"), 
      gst_pipeline_("v4l2src device=/dev/video0 do-timestamp=true ! image/jpeg, width=3840, height=2160, framerate=30/1 ! nvv4l2decoder ! nvvidconv ! video/x-raw, format=I420 ! queue max-size-buffers=1 leaky=downstream ! appsink name=appsink emit-signals=true max-buffers=1 drop=true sync=false async=false",
      std::bind(&CameraPublisher::publish_frame, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)),
      frame_count_(0)
    {
        publisher_ = this->create_publisher<static_image_msgs::msg::Image4k>("camera/image_raw/compressed", rclcpp::QoS(0).best_effort());

        if (!gst_pipeline_.start())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start GStreamer pipeline!");
            rclcpp::shutdown();
            return;
        }
    }

private:
    void publish_frame(uint64_t timestamp, size_t , std::string_view , std::span<unsigned char> frame)
    {
        auto msg = publisher_->borrow_loaned_message();
        msg.get().timestamp = rclcpp::Time(timestamp);
        std::ranges::copy(frame.begin(), frame.end(),  msg.get().data.begin());

        publisher_->publish(std::move(msg));
        auto latency = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - std::chrono::steady_clock::time_point(std::chrono::nanoseconds(timestamp))).count();
        RCLCPP_INFO(this->get_logger(), "Frame latency: %zu ms", latency);
    }

    VideoPipeline gst_pipeline_;
    size_t frame_count_;
    rclcpp::Publisher<static_image_msgs::msg::Image4k>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
