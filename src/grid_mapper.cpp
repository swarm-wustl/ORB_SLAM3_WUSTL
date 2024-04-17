#include <functional>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "vslam/msg/map.hpp"
#include "vslam/msg/map_point.hpp"
#include "vslam/msg/key_frame.hpp"

#include "opencv2/opencv.hpp"

using namespace std;

struct Point {
  int x;
  int y;
};

struct GridCell {
  int nOccupied;
  int nVisited;
};

vector<Point> bresenham(int x1, int y1, int x2, int y2) {
  vector<Point> points;
  bool steep = abs(y2 - y1) > abs(x2 - x1);
  int temp;
  if (steep) {
    temp = x1;
    x1 = y1;
    y1 = temp;

    temp = x2;
    x2 = y2;
    y2 = temp;
  }
  if (x1 > x2) {
    temp = x1;
    x1 = x2;
    x2 = temp;

    temp = y1;
    y1 = y2;
    y2 = temp;
  }
  int dx = x2 - x1;
  int dy = abs(y2 - y1);
  int error = dx / 2;
  int ystep = (y1 < y2) ? 1 : -1;
  int y = y1;

  for (int x = x1; x <= x2; x++) {
    if (steep) {
      points.push_back({y, x});  // Note the order of x and y is swapped back for steep lines
    } else {
      points.push_back({x, y});
    }
    error -= dy;
    if (error < 0) {
      y += ystep;
      error += dx;
    }
  }

  return points;
}

class GridMapperNode : public rclcpp::Node {
  private:
    rclcpp::Subscription<vslam::msg::Map>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    cv::Mat img;
    cv::Mat grid_map_occ;
    cv::Mat grid_map_vis;

    int GRID_SIZE_CM;
    int GRID_RESOLUTION_CM;
    int PIXELS_PER_CELL;
    int MAX_XZ;
    int DISPLAY_GRID_MIN;

    int IMG_SIZE;
    int GRID_SIZE;

    bool display_grid;

    Point xzToGrid(float x, float z) {
      int grid_x = GRID_SIZE / 2 + x / GRID_RESOLUTION_CM * 100;
      int grid_z = GRID_SIZE / 2 + z / GRID_RESOLUTION_CM * 100;
      grid_x = min(max(grid_x, 0), GRID_SIZE - 1);
      grid_z = min(max(grid_z, 0), GRID_SIZE - 1);
      return {grid_x, grid_z};
    }

    void update_grid_map(geometry_msgs::msg::Pose pose, vector<vslam::msg::MapPoint> map_points, vector<vslam::msg::MapPoint> tracked_map_points, vector<vslam::msg::KeyFrame> key_frames) {
      // Cast line from robot to tracked map points
      Point robot_grid_point = xzToGrid(pose.position.x, pose.position.z);
      float MIN_Y = 0;
      // float MAX_Y = 1.5;
      float MAX_Y = 15;

      for (const auto& map_point : tracked_map_points) {
        if (-map_point.world_pos.y < MIN_Y || -map_point.world_pos.y > MAX_Y) {
          continue;
        }
        Point map_point_grid_point = xzToGrid(map_point.world_pos.x, map_point.world_pos.z);
        if (
            map_point_grid_point.x >= GRID_SIZE - 5 ||
            map_point_grid_point.x < 5 ||
            map_point_grid_point.y >= GRID_SIZE - 5 ||
            map_point_grid_point.y < 5 ||
            robot_grid_point.x >= GRID_SIZE - 5 ||
            robot_grid_point.x < 5 ||
            robot_grid_point.y >= GRID_SIZE - 5 ||
            robot_grid_point.y < 5
        ) {
          continue;
        }

        // RCLCPP_INFO(this->get_logger(), "Robot: %d %d, Map: %d %d", robot_grid_point.x, robot_grid_point.y, map_point_grid_point.x, map_point_grid_point.y);
        vector<Point> visited_points = bresenham(robot_grid_point.x, robot_grid_point.y, map_point_grid_point.x, map_point_grid_point.y);
        for (const auto& point : visited_points) {
          grid_map_vis.at<int>(point.x, point.y)++;
        }
        grid_map_occ.at<int>(map_point_grid_point.x, map_point_grid_point.y) += 3;
        grid_map_vis.at<int>(map_point_grid_point.x, map_point_grid_point.y) += 3;
      }
    }

    void display_grid_map(geometry_msgs::msg::Pose pose) {
      img.setTo(cv::Scalar(255, 255, 255));
      int grid_size = GRID_SIZE;
      Point pose_grid_point = xzToGrid(pose.position.x, pose.position.z);

      for (int i = 0; i < grid_size; i++) {
        for (int j = 0; j < grid_size; j++) {
          cv::Scalar color;

          // Check if the current cell corresponds to the pose
          if (i == pose_grid_point.x && j == pose_grid_point.y) {
            color = cv::Scalar(0, 0, 255);
          } else {
            // GridCell cell = grid_map[i][j];
            GridCell cell = {grid_map_occ.at<int>(i, j), grid_map_vis.at<int>(i, j)};
            // Calculate color value based on occupancy
            int color_value = (1 - ((float)cell.nOccupied / (cell.nVisited*0.1 + 1))) * 255;

            // Adjust color if below minimum display threshold
            if (cell.nVisited < DISPLAY_GRID_MIN) {
              color = cv::Scalar(255, 120, 120);
            } else {
              color = cv::Scalar(color_value, color_value, color_value);
            }
          }

          // Draw rectangle with appropriate color
          cv::rectangle(img, cv::Rect(i * PIXELS_PER_CELL, j * PIXELS_PER_CELL, PIXELS_PER_CELL, PIXELS_PER_CELL), color, cv::FILLED);
        }
      }
    }

    void publish_occupancy_map() {
      // Apply a Gaussian blur to the grid map
      cv::Mat processed_occ;
      cv::Mat processed_vis;
      cv::GaussianBlur(grid_map_occ, processed_occ, cv::Size(5, 5), 0);
      cv::GaussianBlur(grid_map_vis, processed_vis, cv::Size(5, 5), 0);

      // Publish occupancy grid map
      nav_msgs::msg::OccupancyGrid grid_msg;
      grid_msg.header.stamp = this->now();
      grid_msg.header.frame_id = "/odom";
      grid_msg.info.resolution = GRID_RESOLUTION_CM / 100.0;
      grid_msg.info.width = GRID_SIZE;
      grid_msg.info.height = GRID_SIZE;
      grid_msg.info.origin.position.x = MAX_XZ/4 + 0.5;
      grid_msg.info.origin.position.y = -MAX_XZ/4 - 0.5;
      grid_msg.info.origin.position.z = 0;
      grid_msg.info.origin.orientation.x = 0;
      grid_msg.info.origin.orientation.y = 0;
      grid_msg.info.origin.orientation.z = 0.7071068;
      grid_msg.info.origin.orientation.w = 0.7071068;

      vector<int8_t> data;
      for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
          // GridCell cell = grid_map[GRID_SIZE - 1 - i][j];
          GridCell cell = {processed_occ.at<int>(GRID_SIZE - 1 - i, j), processed_vis.at<int>(GRID_SIZE - 1 - i, j)};
          int8_t occupancy = cell.nVisited < DISPLAY_GRID_MIN ? -1 : min((1 - ((float)cell.nOccupied / cell.nVisited)) * 100, 90.f);
          data.push_back(occupancy);
        }
      }
      grid_msg.data = data;
      grid_pub_->publish(grid_msg);
    }

    void map_callback(const vslam::msg::Map::SharedPtr msg)
    {
      vector<vslam::msg::MapPoint> map_points = msg->map_points;
      vector<vslam::msg::MapPoint> tracked_map_points = msg->tracked_map_points;
      vector<vslam::msg::KeyFrame> key_frames = msg->key_frames;
      geometry_msgs::msg::Pose pose = msg->camera_pose;
      
      update_grid_map(pose, map_points, tracked_map_points, key_frames);
      publish_occupancy_map();

      // if (display_grid) {
      //   display_grid_map(pose);
      //   cv::imshow("Grid", img);
      //   cv::waitKey(1);
      // }
    }

  public:
    GridMapperNode() : Node("grid_mapper")
  {
    auto display_grid_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    display_grid_param_desc.description = "Display grid map?";
    this->declare_parameter<bool>("display_grid", true, display_grid_param_desc);
    display_grid = this->get_parameter("display_grid").as_bool();

    auto map_topic_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    map_topic_param_desc.description = "Name of the topic to subscribe to for the map";
    this->declare_parameter<string>("map_topic", "vslam/map", map_topic_param_desc);

    auto grid_size_cm_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    grid_size_cm_param_desc.description = "Size of the grid in centimeters";
    this->declare_parameter<int>("grid_size_cm", 500, grid_size_cm_param_desc);

    auto grid_resolution_cm_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    grid_resolution_cm_param_desc.description = "Resolution of the grid in centimeters";
    this->declare_parameter<int>("grid_resolution_cm", 5, grid_resolution_cm_param_desc);

    auto pixels_per_cell_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    pixels_per_cell_param_desc.description = "Number of pixels per cell for image display";
    this->declare_parameter<int>("pixels_per_cell", 4, pixels_per_cell_param_desc);

    auto max_xz_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    max_xz_param_desc.description = "Maximum value of x and z in the world";
    this->declare_parameter<int>("max_xz", 10, max_xz_param_desc);

    auto display_grid_min_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    display_grid_min_param_desc.description = "Minimum number of points a cell needs to be visible at all";
    this->declare_parameter<int>("display_grid_min", 3, display_grid_min_param_desc);

    GRID_SIZE_CM = this->get_parameter("grid_size_cm").as_int();
    GRID_RESOLUTION_CM = this->get_parameter("grid_resolution_cm").as_int();
    PIXELS_PER_CELL = this->get_parameter("pixels_per_cell").as_int();
    MAX_XZ = this->get_parameter("max_xz").as_int();
    DISPLAY_GRID_MIN = this->get_parameter("display_grid_min").as_int();
    IMG_SIZE = GRID_SIZE_CM / GRID_RESOLUTION_CM * PIXELS_PER_CELL;
    GRID_SIZE = GRID_SIZE_CM / GRID_RESOLUTION_CM;

    // Subscribe to point cloud and pose
    string map_topic = this->get_parameter("map_topic").as_string();
    map_sub_ = this->create_subscription<vslam::msg::Map>(map_topic, 10, bind(&GridMapperNode::map_callback, this, placeholders::_1));
    img = cv::Mat::zeros(IMG_SIZE, IMG_SIZE, CV_8UC3);

    // Initialize grid map
    grid_map_occ = cv::Mat::zeros(GRID_SIZE, GRID_SIZE, CV_16U);
    grid_map_vis = cv::Mat::zeros(GRID_SIZE, GRID_SIZE, CV_16U);

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 10);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GridMapperNode initialized");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMapperNode>());
  rclcpp::shutdown();
  return 0;
}

