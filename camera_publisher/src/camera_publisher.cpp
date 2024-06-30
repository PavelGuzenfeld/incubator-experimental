#include "video_pipeline.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
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
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_raw/compressed", rclcpp::QoS(0).best_effort());

        if (!gst_pipeline_.start())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start GStreamer pipeline!");
            rclcpp::shutdown();
            return;
        }
    }

private:
    void publish_frame(uint64_t timestamp, size_t frame_size, std::string_view frame_format, std::span<unsigned char> frame)
    {
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = rclcpp::Time(timestamp);
        msg.format = std::string(frame_format);
        // msg.data.assign(frame.begin(), frame.end());

        msg.data.reserve(frame_size);
        msg.data.insert(msg.data.end(), frame.begin(), frame.end());

        publisher_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Published frame %zu", frame_count_++);
    }

    VideoPipeline gst_pipeline_;
    size_t frame_count_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
