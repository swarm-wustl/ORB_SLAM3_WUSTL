#include <functional>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"


#include "vslam/msg/map.hpp"
#include "vslam/msg/map_point.hpp"
#include "vslam/msg/key_frame.hpp"

#include "opencv2/opencv.hpp"

using namespace std;

class MapVisualizationSplitter : public rclcpp::Node {
  private:
    rclcpp::Subscription<vslam::msg::Map>::SharedPtr map_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_points_pub_;

    void map_callback(const vslam::msg::Map::SharedPtr msg)
    {
      vector<vslam::msg::MapPoint> map_points = msg->map_points;
      vector<vslam::msg::MapPoint> tracked_points = msg->tracked_map_points;
      vector<vslam::msg::KeyFrame> key_frames = msg->key_frames;
      RCLCPP_INFO(this->get_logger(), "Received map with %d map points and %d key frames", map_points.size(), key_frames.size());

      // Visualize camera pose
      geometry_msgs::msg::Pose pose = msg->camera_pose;

      // Swap y and z axes
      double y = pose.position.y;
      pose.position.y = pose.position.z;
      pose.position.z = y;

      // Swap pitch and yaw and subtract 90 degrees to yaw
      tf2::Quaternion q;
      tf2::fromMsg(pose.orientation, q);
      tf2::Quaternion transformation(0.5, 0.5, -0.5, -0.5);
      q = transformation*q;
      pose.orientation = tf2::toMsg(q);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      pose_stamped.header.stamp = this->now();
      pose_stamped.header.frame_id = "/odom";
      pose_pub_->publish(pose_stamped);

      // Visualize odometry
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = this->now();
      odometry.header.frame_id = "/odom";
      odometry.child_frame_id = "/base_link";

      odometry.pose.pose = pose;
      odometry.twist.twist.linear.x = 0.0;
      odometry.twist.twist.linear.y = 0.0;
      odometry.twist.twist.linear.z = 0.0;
      odometry.twist.twist.angular.x = 0.0;
      odometry.twist.twist.angular.y = 0.0;
      odometry.twist.twist.angular.z = 0.0;

      odometry_pub_->publish(odometry);

      // Visualize map points in rviz
      sensor_msgs::msg::PointCloud2 map_points_msg;
      map_points_msg.header.stamp = this->now();
      map_points_msg.header.frame_id = "/odom";
      map_points_msg.height = 1;
      map_points_msg.width = map_points.size();
      map_points_msg.is_dense = false;
      map_points_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier(map_points_msg);
      modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::msg::PointField::FLOAT32);
      sensor_msgs::PointCloud2Iterator<float> iter_x(map_points_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(map_points_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(map_points_msg, "z");

      modifier.resize(map_points.size());
      for (int i = 0; i < map_points.size(); i++)
      {
        vslam::msg::MapPoint map_point = map_points[i];
        double max_val = 50.0f;
        *iter_x = max(min(map_point.world_pos.x, max_val), -max_val);
        *iter_y = max(min(map_point.world_pos.z, max_val), -max_val); // swap y and z
        *iter_z = max(min(-map_point.world_pos.y, max_val), -max_val); // swap y and z
        ++iter_x; ++iter_y; ++iter_z;
      }

      point_cloud_pub_->publish(map_points_msg);

      // Visualize tracked points in rviz
      sensor_msgs::msg::PointCloud2 tracked_points_msg;
      tracked_points_msg.header.stamp = this->now();
      tracked_points_msg.header.frame_id = "/odom";
      tracked_points_msg.height = 1;
      tracked_points_msg.width = tracked_points.size();
      tracked_points_msg.is_dense = false;
      tracked_points_msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier_tracked(tracked_points_msg);
      modifier_tracked.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                               "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                               "z", 1, sensor_msgs::msg::PointField::FLOAT32);
      sensor_msgs::PointCloud2Iterator<float> iter_x_tracked(tracked_points_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y_tracked(tracked_points_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z_tracked(tracked_points_msg, "z");

      modifier_tracked.resize(tracked_points.size());
      for (int i = 0; i < tracked_points.size(); i++)
      {
        vslam::msg::MapPoint map_point = tracked_points[i];
        double max_val = 50.0f;
        *iter_x_tracked = max(min(map_point.world_pos.x, max_val), -max_val);
        *iter_y_tracked = max(min(map_point.world_pos.z, max_val), -max_val); // swap y and z
        *iter_z_tracked = max(min(-map_point.world_pos.y, max_val), -max_val); // swap y and z
        ++iter_x_tracked; ++iter_y_tracked; ++iter_z_tracked;
      }

      tracked_points_pub_->publish(tracked_points_msg);
    }

  public:
    MapVisualizationSplitter() : Node("map_visualization_splitter")
  {
    auto map_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    map_topic_param_desc.description = "Name of the topic to subscribe to for the map";
    this->declare_parameter<string>("map_topic", "vslam/map", map_topic_param_desc);

    // Subscribe to map
    string map_topic = this->get_parameter("map_topic").as_string();
    map_sub_ = this->create_subscription<vslam::msg::Map>(map_topic, 10, bind(&MapVisualizationSplitter::map_callback, this, placeholders::_1));

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vslam/pose", 1000);
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("vslam/odometry", 1000);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/map_points", 1000);
    tracked_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/tracked_points", 1000);
    RCLCPP_INFO(this->get_logger(), "MapVisualizationSplitter node initialized");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizationSplitter>());
  rclcpp::shutdown();
  return 0;
}


