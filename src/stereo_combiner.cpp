#include <functional>
#include <chrono>
#include <string>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "vslam/msg/stereo_image.hpp"

using namespace std;

cv_bridge::CvImagePtr getImageFromMessage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e) {
    cerr << "cv_bridge exception: " << e.what() << endl;
    return nullptr;
  }
  return cv_ptr;
}

class StereoCombinerNode : public rclcpp::Node {
  private:
    rclcpp::Clock shared_clock();
    double last_left_callback;
    double last_right_callback;
    image_transport::Subscriber left_image_sub_;
    image_transport::Subscriber right_image_sub_;
    int allowed_latency;

    cv::Mat last_right_img;
    rclcpp::Publisher<vslam::msg::StereoImage>::SharedPtr stereo_image_pub_;

    void leftImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      cv_bridge::CvImagePtr cv_ptr = getImageFromMessage(msg);
      if (!cv_ptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image message to cv_bridge");
        return;
      }

      cv_ptr->image.empty(); // required to prevent segfault

      // Check that images were received within 50ms of each other
      if (this->now().seconds() - last_right_callback > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Received left image but right image was not received within 25ms");
        return;
      }

      vslam::msg::StereoImage::SharedPtr stereo_img_msg = std::make_shared<vslam::msg::StereoImage>();
      stereo_img_msg->header.stamp = this->now();
      stereo_img_msg->left_image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_ptr->image).toImageMsg();
      stereo_img_msg->right_image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", last_right_img).toImageMsg();
      stereo_image_pub_->publish(*stereo_img_msg);

      RCLCPP_INFO(this->get_logger(), "Published stereo image");
    }

    void rightImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      cv_bridge::CvImagePtr cv_ptr = getImageFromMessage(msg);
      if (!cv_ptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image message to cv_bridge");
        return;
      }

      cv_ptr->image.empty(); // required to prevent segfault
      last_right_img = cv_ptr->image.clone();
      last_right_callback = this->now().seconds();
    }

  public:
    StereoCombinerNode() : Node("stereo_combiner_node")
  {
    auto left_video_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    left_video_topic_param_desc.description = "Name of the topic to subscribe to for the left video feed";
    this->declare_parameter<string>("left_video_topic", "left_camera/image_raw", left_video_topic_param_desc);

    auto right_video_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    right_video_topic_param_desc.description = "Name of the topic to subscribe to for the right video feed";
    this->declare_parameter<string>("right_video_topic", "right_camera/image_raw", right_video_topic_param_desc);

    auto allowed_latency_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    allowed_latency_topic_param_desc.description = "Allowed latency in ms between left and right camera feeds";
    this->declare_parameter<int>("allowed_latency", 100, allowed_latency_topic_param_desc);
    allowed_latency = this->get_parameter("allowed_latency").as_int();

    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    left_image_sub_ = image_transport::create_subscription(
        this,
        this->get_parameter("left_video_topic").as_string(),
        bind(&StereoCombinerNode::leftImageCallback, this, placeholders::_1),
        "raw",
        custom_qos
    );
    right_image_sub_ = image_transport::create_subscription(
        this,
        this->get_parameter("right_video_topic").as_string(),
        bind(&StereoCombinerNode::rightImageCallback, this, placeholders::_1),
        "raw",
        custom_qos
    );

    stereo_image_pub_ = this->create_publisher<vslam::msg::StereoImage>("vslam/combined", 10);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCombinerNode>());
  rclcpp::shutdown();
  return 0;
}


