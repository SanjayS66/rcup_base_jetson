#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Replace with your actual package name
#include "laser_line_extraction_ros2/msg/line_segment_list.hpp"

using namespace std::placeholders;

class SimpleAligner : public rclcpp::Node
{
public:
  SimpleAligner() : Node("simple_aligner")
  {
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("target_distance", 0.05);
    this->declare_parameter("max_linear_velocity", 0.075);
    this->declare_parameter("max_angular_velocity", 0.1);
    this->declare_parameter("lin_p", 0.8);
    this->declare_parameter("lin_i", 0.0);
    this->declare_parameter("lin_d", 0.0);
    this->declare_parameter("ang_p", 0.5);
    this->declare_parameter("ang_i", 0.0);
    this->declare_parameter("ang_d", 0.0);
    this->declare_parameter("error_tolerance_lin", 0.02);
    this->declare_parameter("error_tolerance_ang", 0.05);
    this->declare_parameter("laser_axis", "+x");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);
      
    line_sub_ = this->create_subscription<laser_line_extraction_ros2::msg::LineSegmentList>(
      "line_segments", 10, std::bind(&SimpleAligner::lineCallback, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "Simple Aligner Started. Aligning and will stop when done.");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<laser_line_extraction_ros2::msg::LineSegmentList>::SharedPtr line_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double err_lin_int_ = 0.0, err_ang_int_ = 0.0;
  double last_err_lin_ = 0.0, last_err_ang_ = 0.0;

  void lineCallback(const laser_line_extraction_ros2::msg::LineSegmentList::SharedPtr msg)
  {
    if (msg->line_segments.empty()) return;

    // 1. Get the closest line
    auto closest_line = msg->line_segments[0];
    for (const auto& line : msg->line_segments) {
      if (line.radius < closest_line.radius) closest_line = line;
    }

    // 2. Transform endpoints to base_link
    geometry_msgs::msg::PointStamped p1_in, p2_in, p1_out, p2_out;
    p1_in.header.frame_id = msg->header.frame_id;
    p1_in.header.stamp = msg->header.stamp;
    p1_in.point.x = closest_line.start[0]; p1_in.point.y = closest_line.start[1];
    
    p2_in = p1_in;
    p2_in.point.x = closest_line.end[0]; p2_in.point.y = closest_line.end[1];

    try {
      p1_out = tf_buffer_->transform(p1_in, "base_link", tf2::durationFromSec(0.1));
      p2_out = tf_buffer_->transform(p2_in, "base_link", tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
      return;
    }

    // 3. Calculate Geometry
    double dx = p2_out.point.x - p1_out.point.x;
    double dy = p2_out.point.y - p1_out.point.y;
    double line_angle = std::atan2(dy, dx);
    
    double num = std::abs(dx * p1_out.point.y - p1_out.point.x * dy);
    double den = std::hypot(dx, dy);
    double current_distance = num / den;

    std::string axis = this->get_parameter("laser_axis").as_string();
    double target_dist = this->get_parameter("target_distance").as_double();
    
    double error_lin = current_distance - target_dist;
    double error_ang = 0.0;

    if (axis == "+x" || axis == "-x") {
        error_ang = std::atan(std::tan(line_angle - M_PI_2)); 
    } else if (axis == "+y" || axis == "-y") {
        error_ang = std::atan(std::tan(line_angle)); 
    }

    // 4. PID calculation
    double kp_l = this->get_parameter("lin_p").as_double();
    double ki_l = this->get_parameter("lin_i").as_double();
    double kd_l = this->get_parameter("lin_d").as_double();
    
    err_lin_int_ = std::clamp(err_lin_int_ + error_lin, -0.1, 0.1);
    double err_lin_d = error_lin - last_err_lin_;
    last_err_lin_ = error_lin;
    double vel_lin = (kp_l * error_lin) + (ki_l * err_lin_int_) + (kd_l * err_lin_d);

    double kp_a = this->get_parameter("ang_p").as_double();
    double ki_a = this->get_parameter("ang_i").as_double();
    double kd_a = this->get_parameter("ang_d").as_double();
    
    err_ang_int_ = std::clamp(err_ang_int_ + error_ang, -0.1, 0.1);
    double err_ang_d = error_ang - last_err_ang_;
    last_err_ang_ = error_ang;
    double vel_ang = (kp_a * error_ang) + (ki_a * err_ang_int_) + (kd_a * err_ang_d);

    double max_v = this->get_parameter("max_linear_velocity").as_double();
    double max_w = this->get_parameter("max_angular_velocity").as_double();
    vel_lin = std::clamp(vel_lin, -max_v, max_v);
    vel_ang = std::clamp(vel_ang, -max_w, max_w);

    // 5. Publish to specific mecanum axis
    geometry_msgs::msg::Twist cmd;
    if (axis == "+x") cmd.linear.x = vel_lin;
    else if (axis == "-x") cmd.linear.x = -vel_lin;
    else if (axis == "+y") cmd.linear.y = vel_lin;
    else if (axis == "-y") cmd.linear.y = -vel_lin;
    cmd.angular.z = vel_ang;

    cmd_pub_->publish(cmd);

    // 6. Stop and Exit Condition
    if (std::abs(error_lin) < this->get_parameter("error_tolerance_lin").as_double() &&
        std::abs(error_ang) < this->get_parameter("error_tolerance_ang").as_double())
    {
      RCLCPP_INFO(this->get_logger(), "Alignment Complete. Stopping robot and exiting.");
      geometry_msgs::msg::Twist stop_cmd;
      cmd_pub_->publish(stop_cmd);
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleAligner>());
  return 0;
}