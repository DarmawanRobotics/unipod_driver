// One UniPod RTSP stream as a ROS image topic. Run one per stream: video1 (main) and video2
// (sub); what each carries is set with the driver's main_stream / sub_stream.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "unipod_driver/rtsp_capture.hpp"

namespace unipod_driver
{

class UnipodCameraNode : public rclcpp::Node
{
public:
  explicit UnipodCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera", options)
  {
    CaptureConfig c;
    c.url = declare_parameter("url", c.url);
    c.codec = declare_parameter("codec", c.codec);
    c.decoder = declare_parameter("decoder", c.decoder);
    c.latency_ms = static_cast<int>(declare_parameter("latency_ms", c.latency_ms));
    c.protocols = declare_parameter("protocols", c.protocols);
    c.reconnect_sec = declare_parameter("reconnect_sec", c.reconnect_sec);
    c.timeout_ms = static_cast<int>(declare_parameter("timeout_ms", c.timeout_ms));
    frame_id_ = declare_parameter("frame_id", "unipod_camera");
    encoding_ = declare_parameter("encoding", "bgr8");
    max_fps_ = declare_parameter("max_fps", 0.0);
    width_ = static_cast<int>(declare_parameter("width", 0));
    height_ = static_cast<int>(declare_parameter("height", 0));

    if (encoding_ != "bgr8" && encoding_ != "mono8") {
      throw std::invalid_argument("encoding must be bgr8 or mono8, not '" + encoding_ + "'");
    }

    pub_ = image_transport::create_publisher(this, "image_raw");
    capture_ = std::make_unique<RtspCapture>(
      c, [this](const cv::Mat & f) {onFrame(f);},
      [this](const std::string & m) {RCLCPP_INFO(get_logger(), "%s", m.c_str());});
    capture_->start();
  }

  ~UnipodCameraNode() override
  {
    if (capture_) {
      capture_->stop();
    }
  }

private:
  void onFrame(const cv::Mat & frame)
  {
    const auto stamp = now();
    const auto t = std::chrono::steady_clock::now();
    if (max_fps_ > 0.0 && t - last_pub_ < std::chrono::duration<double>(1.0 / max_fps_)) {
      return;
    }
    if (pub_.getNumSubscribers() == 0) {
      return;  // decode keeps running (newest frame only), but skip the conversion
    }
    last_pub_ = t;

    cv::Mat out = frame;
    if (width_ > 0 && height_ > 0 && (frame.cols != width_ || frame.rows != height_)) {
      cv::resize(frame, out, cv::Size(width_, height_), 0, 0, cv::INTER_AREA);
    }
    if (encoding_ == "mono8") {
      cv::Mat gray;
      cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
      out = gray;
    }
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;
    pub_.publish(cv_bridge::CvImage(header, encoding_, out).toImageMsg());
  }

  std::string frame_id_;
  std::string encoding_;
  double max_fps_{0.0};
  int width_{0};
  int height_{0};
  std::chrono::steady_clock::time_point last_pub_{};
  image_transport::Publisher pub_;
  std::unique_ptr<RtspCapture> capture_;
};

}  // namespace unipod_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<unipod_driver::UnipodCameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("unipod_camera"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
