#include <functional>
#include <chrono>
#include <string>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "vslam/msg/map.hpp"
#include "vslam/msg/key_frame.hpp"
#include "vslam/msg/map_point.hpp"
#include "vslam/msg/stereo_image.hpp"

#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include "System.h" // ORB_SLAM3
#include "ImuTypes.h" // ORB_SLAM3

using namespace std;

bool throttleCallback(const chrono::time_point<chrono::high_resolution_clock>& last_callback) {
  int FPS = 10;
  auto now = chrono::high_resolution_clock::now();
  if (now - last_callback < chrono::milliseconds(1000 / FPS)) {
    return true;
  }
  return false;
}

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

void processImage(cv::Mat& image) {
  cv::resize(image, image, cv::Size(752, 480));
}

class VSLAMSystemNode : public rclcpp::Node {
  private:
    const string OPENCV_WINDOW = "Camera Feed";
    const int DIST_MULT = 5;
    chrono::time_point<chrono::high_resolution_clock> last_callback;

    image_transport::Subscriber mono_image_sub_;
    rclcpp::Subscription<vslam::msg::StereoImage>::SharedPtr stereo_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<vslam::msg::Map>::SharedPtr map_publisher_;

    // sensor_msgs::msg::Imu last_imu;
    vector<ORB_SLAM3::IMU::Point> imu_buffer;
    bool usingImu;

    ORB_SLAM3::System* SLAM;

    void processKeyFrame(ORB_SLAM3::KeyFrame* keyframe, vslam::msg::Map::SharedPtr& map_msg) {
      Sophus::SE3f keyframe_pose = keyframe->GetPose();
      Eigen::Vector3f position = keyframe_pose.translation();
      Eigen::Quaternionf orientation = keyframe_pose.unit_quaternion();
      vslam::msg::KeyFrame keyframe_msg;
      keyframe_msg.id = keyframe->mnId;
      keyframe_msg.pose.position.x = position.x() * DIST_MULT;
      keyframe_msg.pose.position.y = position.y() * DIST_MULT;
      keyframe_msg.pose.position.z = position.z() * DIST_MULT;
      keyframe_msg.pose.orientation.x = orientation.x();
      keyframe_msg.pose.orientation.y = orientation.y();
      keyframe_msg.pose.orientation.z = orientation.z();
      keyframe_msg.pose.orientation.w = orientation.w();

      set<ORB_SLAM3::MapPoint*> keyframe_map_points = keyframe->GetMapPoints();
      for (ORB_SLAM3::MapPoint* map_point : keyframe_map_points) {
        keyframe_msg.map_point_ids.push_back(map_point->mnId);
      }

      map_msg->key_frames.push_back(keyframe_msg);
    }

    void processMapPoint(ORB_SLAM3::MapPoint* map_point, vector<vslam::msg::MapPoint>& map_points) {
      if (map_point == nullptr || map_point->isBad()) {
        return;
      }
      vslam::msg::MapPoint map_point_msg;
      Eigen::Vector3f position = map_point->GetWorldPos();
      map_point_msg.id = map_point->mnId;
      map_point_msg.world_pos.x = position.x() * DIST_MULT;
      map_point_msg.world_pos.y = position.y() * DIST_MULT;
      map_point_msg.world_pos.z = position.z() * DIST_MULT;
      map_points.push_back(map_point_msg);
    }

    void publishMap(const Sophus::SE3f& pose) {
      Sophus::SE3f ipose = pose.inverse();
      ORB_SLAM3::Atlas* atlas = SLAM->GetAtlas();
      ORB_SLAM3::Map* map = atlas->GetCurrentMap();
      vector<ORB_SLAM3::KeyFrame*> keyframes = map->GetAllKeyFrames();
      vector<ORB_SLAM3::MapPoint*> all_map_points = map->GetAllMapPoints();
      vector<ORB_SLAM3::MapPoint*> tracked_map_points = SLAM->GetTrackedMapPoints();

      vslam::msg::Map::SharedPtr map_msg = std::make_shared<vslam::msg::Map>();
      map_msg->header.stamp = this->now();
      map_msg->header.frame_id = "/base_link";
      map_msg->id = map->GetId();

      // Set camera pose
      Eigen::Vector3f camera_position = ipose.translation();
      Eigen::Quaternionf camera_orientation = ipose.unit_quaternion();

      map_msg->camera_pose.position.x = camera_position.x() * DIST_MULT;
      map_msg->camera_pose.position.y = camera_position.y() * DIST_MULT;
      map_msg->camera_pose.position.z = camera_position.z() * DIST_MULT;
      map_msg->camera_pose.orientation.x = camera_orientation.x();
      map_msg->camera_pose.orientation.y = camera_orientation.y();
      map_msg->camera_pose.orientation.z = camera_orientation.z();
      map_msg->camera_pose.orientation.w = camera_orientation.w();

      for (ORB_SLAM3::KeyFrame* keyframe : keyframes) {
        processKeyFrame(keyframe, map_msg);
      }

      for (ORB_SLAM3::MapPoint* map_point : all_map_points) {
        processMapPoint(map_point, map_msg->map_points);
      }

      for (ORB_SLAM3::MapPoint* map_point : tracked_map_points) {
        processMapPoint(map_point, map_msg->tracked_map_points);
      }

      map_publisher_->publish(*map_msg);
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      if (throttleCallback(last_callback)) return;

      cv_bridge::CvImagePtr cv_ptr = getImageFromMessage(msg);
      if (!cv_ptr) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image message to cv_bridge");
        return;
      }

      cv_ptr->image.empty(); // required to prevent segfault

      processImage(cv_ptr->image);

      double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0;
      Sophus::SE3f pose;
      if (usingImu) {
        if (imu_buffer.size() < 5) {
          RCLCPP_WARN(this->get_logger(), "No IMU data available for this frame");
        } else {
          pose = SLAM->TrackMonocular(cv_ptr->image, tframe, imu_buffer);
        }
        imu_buffer.clear();
      } else {
        pose = SLAM->TrackMonocular(cv_ptr->image, tframe);
      }

      publishMap(pose);

      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }

    void stereoCallback(const vslam::msg::StereoImage::SharedPtr msg) {
      // if (throttleCallback(last_callback)) return;
      RCLCPP_INFO(this->get_logger(), "Received stereo image");

      cv_bridge::CvImagePtr cv_ptr_l;
      cv_bridge::CvImagePtr cv_ptr_r;
      try {
        cv_ptr_l = cv_bridge::toCvCopy(msg->left_image, msg->left_image.encoding);
        cv_ptr_r = cv_bridge::toCvCopy(msg->right_image, msg->right_image.encoding);
      } catch (cv_bridge::Exception& e) {
        cerr << "cv_bridge exception: " << e.what() << endl;
        return;
      }

      cv_ptr_l->image.empty(); // required to prevent segfault
      cv_ptr_r->image.empty(); // required to prevent segfault

      processImage(cv_ptr_l->image);
      processImage(cv_ptr_r->image);

      double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0;
      Sophus::SE3f pose = SLAM->TrackStereo(cv_ptr_l->image, cv_ptr_r->image, tframe);

      publishMap(pose);

      // Display side by side images
      cv::Mat combined(cv_ptr_l->image.rows, cv_ptr_l->image.cols*2, cv_ptr_l->image.type());
      cv_ptr_l->image.copyTo(combined(cv::Rect(0, 0, cv_ptr_l->image.cols, cv_ptr_l->image.rows)));
      cv_ptr_r->image.copyTo(combined(cv::Rect(cv_ptr_l->image.cols, 0, cv_ptr_r->image.cols, cv_ptr_r->image.rows)));

      cv::imshow(OPENCV_WINDOW, combined);
      cv::waitKey(3);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      cv::Point3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      cv::Point3f gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
      double tframe = msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0;
      ORB_SLAM3::IMU::Point imu_point(acc, gyro, tframe);
      imu_buffer.push_back(imu_point);
    }

  public:
    VSLAMSystemNode() : Node("vslam_system_node")
  {
    auto vocab_path_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    vocab_path_param_desc.description = "Path to the ORB vocabulary";
    this->declare_parameter<string>("vocab_path", "./src/vslam/settings/ORBvoc.txt", vocab_path_param_desc);

    auto settings_path_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    settings_path_param_desc .description = "Path to the camera settings file";
    this->declare_parameter<string>("settings_path", "./src/vslam/settings/mono_settings.yaml", settings_path_param_desc );

    auto display_visual_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    display_visual_param_desc.description = "Display the visual output of the SLAM system";
    this->declare_parameter<bool>("display_visual", true, display_visual_param_desc);

    auto mono_video_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    mono_video_topic_param_desc.description = "Name of the topic to subscribe to for the monocular video feed";
    this->declare_parameter<string>("mono_video_topic", "camera/image_raw", mono_video_topic_param_desc);

    auto stereo_video_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    stereo_video_topic_param_desc.description = "Name of the topic to subscribe to for the stereo video feed";
    this->declare_parameter<string>("stereo_video_topic", "vslam/combined", stereo_video_topic_param_desc);

    auto imu_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    imu_topic_param_desc.description = "Name of the topic to subscribe to for the stereo video feed";
    // this->declare_parameter<string>("imu_topic", "imu_plugin/out", imu_topic_param_desc);
    this->declare_parameter<string>("imu_topic", "null", imu_topic_param_desc);
    usingImu = this->get_parameter("imu_topic").as_string() != "null";

    cv::namedWindow(OPENCV_WINDOW);

    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    string mono_video_topic = this->get_parameter("mono_video_topic").as_string();
    string stereo_video_topic = this->get_parameter("stereo_video_topic").as_string();

    if (mono_video_topic != "null") {
      RCLCPP_INFO(this->get_logger(), "Subscribing to monocular video feed at %s", mono_video_topic.c_str());
      mono_image_sub_ = image_transport::create_subscription(
          this,
          mono_video_topic,
          bind(&VSLAMSystemNode::imageCallback, this, placeholders::_1),
          "raw",
          custom_qos
      );
    } else {
      RCLCPP_INFO(this->get_logger(), "Subscribing to stereo video feed at %s", stereo_video_topic.c_str());
      stereo_image_sub_  = this->create_subscription<vslam::msg::StereoImage>(
          stereo_video_topic,
          10,
          bind(&VSLAMSystemNode::stereoCallback, this, placeholders::_1)
      );
    }
    if (usingImu) {
      RCLCPP_INFO(this->get_logger(), "Subscribing to IMU feed at %s", this->get_parameter("imu_topic").as_string().c_str());
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          this->get_parameter("imu_topic").as_string(),
          10,
          bind(&VSLAMSystemNode::imuCallback, this, placeholders::_1)
      );
    }

    map_publisher_ = this->create_publisher<vslam::msg::Map>("vslam/map", 10);

    string vocab_path = this->get_parameter("vocab_path").as_string();
    if (access(vocab_path.c_str(), F_OK) == -1) {
      RCLCPP_FATAL(this->get_logger(), "Vocabulary file does not exist at %s. Try running `cp ~/Dev/ORB_SLAM3/Vocabulary/ORBvoc.txt %s`", vocab_path.c_str(), vocab_path.c_str());
      return;
    }

    string settings_path = this->get_parameter("settings_path").as_string();
    if (access(settings_path.c_str(), F_OK) == -1) {
      RCLCPP_FATAL(this->get_logger(), "Settings file does not exist at %s", settings_path.c_str());
      return;
    }

    ORB_SLAM3::System::eSensor sensor;
    if (this->get_parameter("stereo_video_topic").as_string() != "null") {
      sensor = usingImu ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO;
    } else {
      sensor = usingImu ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR;
    }

    SLAM = new ORB_SLAM3::System(
      vocab_path,
      settings_path,
      sensor,
      this->get_parameter("display_visual").as_bool()
    );

    RCLCPP_INFO(this->get_logger(), "VSLAMSystemNode initialized. Using stereo: %d. Using IMU: %d", stereo_video_topic != "null", usingImu);
  }

    ~VSLAMSystemNode()
    {
      cv::destroyWindow(OPENCV_WINDOW);
      SLAM->Shutdown();
    }

  private:
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VSLAMSystemNode>());
  rclcpp::shutdown();
  return 0;
}

