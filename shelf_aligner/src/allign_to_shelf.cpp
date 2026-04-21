#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "laser_line_extraction_ros2/msg/line_segment_list.hpp"

using namespace std::placeholders;

class SimpleAligner : public rclcpp::Node
{
public:
  SimpleAligner() : Node("simple_aligner")
  {
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter("target_distance", 0.55);
    this->declare_parameter("max_linear_velocity", 0.05);
    this->declare_parameter("max_angular_velocity", 0.1);
    
    // PID Parameters
    this->declare_parameter("lin_p", 0.8);
    this->declare_parameter("lin_i", 0.0);
    this->declare_parameter("lin_d", 0.0);
    this->declare_parameter("ang_p", 0.5);
    this->declare_parameter("ang_i", 0.0);
    this->declare_parameter("ang_d", 0.0);
    this->declare_parameter("strafe_p", 0.8);
    this->declare_parameter("strafe_i", 0.0);
    this->declare_parameter("strafe_d", 0.0);

    // Tolerances
    this->declare_parameter("error_tolerance_lin", 0.05);
    this->declare_parameter("error_tolerance_ang", 0.03);
    this->declare_parameter("error_tolerance_strafe", 0.05);
    this->declare_parameter("laser_axis", "+x");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);
      
    line_sub_ = this->create_subscription<laser_line_extraction_ros2::msg::LineSegmentList>(
      "line_segments", 10, std::bind(&SimpleAligner::lineCallback, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "Simple Aligner Started. Aligning (Dist, Ang, Center).");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<laser_line_extraction_ros2::msg::LineSegmentList>::SharedPtr line_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double err_lin_int_ = 0.0, err_ang_int_ = 0.0, err_strafe_int_ = 0.0;
  double last_err_lin_ = 0.0, last_err_ang_ = 0.0, last_err_strafe_ = 0.0;

  void lineCallback(const laser_line_extraction_ros2::msg::LineSegmentList::SharedPtr msg)
  {
    if (msg->line_segments.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No lines found!");
      return;
    }

    auto closest_line = msg->line_segments[0];
    for (const auto& line : msg->line_segments) {
      if (line.radius < closest_line.radius) closest_line = line;
    }

    geometry_msgs::msg::PointStamped p1_in, p2_in, p1_out, p2_out;
    p1_in.header.frame_id = msg->header.frame_id;
    p1_in.header.stamp = msg->header.stamp;
    p1_in.point.x = closest_line.start[0]; p1_in.point.y = closest_line.start[1];
    
    p2_in = p1_in;
    p2_in.point.x = closest_line.end[0]; p2_in.point.y = closest_line.end[1];

    try {
      geometry_msgs::msg::TransformStamped t_stamped = tf_buffer_->lookupTransform(
        "base_link", msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.1));
      tf2::doTransform(p1_in, p1_out, t_stamped);
      tf2::doTransform(p2_in, p2_out, t_stamped);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Error: %s", ex.what());
      return;
    }

    // Geometry Calculations
    double dx = p2_out.point.x - p1_out.point.x;
    double dy = p2_out.point.y - p1_out.point.y;
    double line_angle = std::atan2(dy, dx);
    
    double num = std::abs(dx * p1_out.point.y - p1_out.point.x * dy);
    double den = std::hypot(dx, dy);
    double current_distance = num / den;

    // Midpoint Calculation for centering
    double mid_x = (p1_out.point.x + p2_out.point.x) / 2.0;
    double mid_y = (p1_out.point.y + p2_out.point.y) / 2.0;

    std::string axis = this->get_parameter("laser_axis").as_string();
    double target_dist = this->get_parameter("target_distance").as_double();
    
    double error_lin = current_distance - target_dist;
    double error_ang = 0.0;
    double error_strafe = 0.0;

    // Axis logic
    if (axis == "+x" || axis == "-x") {
        error_ang = std::atan(std::tan(line_angle - M_PI_2)); 
        error_strafe = mid_y; // Center offset is along Y axis
    } else if (axis == "+y" || axis == "-y") {
        error_ang = std::atan(std::tan(line_angle)); 
        error_strafe = mid_x; // Center offset is along X axis
    }

    // Controllers
    double max_v = this->get_parameter("max_linear_velocity").as_double();
    double max_w = this->get_parameter("max_angular_velocity").as_double();

    // 1. Linear PID
    err_lin_int_ = std::clamp(err_lin_int_ + error_lin, -0.1, 0.1);
    double err_lin_d = error_lin - last_err_lin_;
    last_err_lin_ = error_lin;
    double vel_lin = (this->get_parameter("lin_p").as_double() * error_lin) + 
                     (this->get_parameter("lin_i").as_double() * err_lin_int_) + 
                     (this->get_parameter("lin_d").as_double() * err_lin_d);
    vel_lin = std::clamp(vel_lin, -max_v, max_v);

    // 2. Angular PID
    err_ang_int_ = std::clamp(err_ang_int_ + error_ang, -0.1, 0.1);
    double err_ang_d = error_ang - last_err_ang_;
    last_err_ang_ = error_ang;
    double vel_ang = (this->get_parameter("ang_p").as_double() * error_ang) + 
                     (this->get_parameter("ang_i").as_double() * err_ang_int_) + 
                     (this->get_parameter("ang_d").as_double() * err_ang_d);
    vel_ang = std::clamp(vel_ang, -max_w, max_w);

    // 3. Strafe PID
    err_strafe_int_ = std::clamp(err_strafe_int_ + error_strafe, -0.1, 0.1);
    double err_strafe_d = error_strafe - last_err_strafe_;
    last_err_strafe_ = error_strafe;
    double vel_strafe = (this->get_parameter("strafe_p").as_double() * error_strafe) + 
                        (this->get_parameter("strafe_i").as_double() * err_strafe_int_) + 
                        (this->get_parameter("strafe_d").as_double() * err_strafe_d);
    vel_strafe = std::clamp(vel_strafe, -max_v, max_v);

    // Debugging Output
    RCLCPP_INFO(this->get_logger(), 
      "Err [Lin: %.3f | Ang: %.3f | Strf: %.3f] -> CMD [Vx: %.3f | Vy: %.3f | Wz: %.3f]", 
      error_lin, error_ang, error_strafe, vel_lin, vel_strafe, vel_ang);

    // Command Mapping
    geometry_msgs::msg::Twist cmd;
    if (axis == "+x") {
        cmd.linear.x = vel_lin;
        cmd.linear.y = vel_strafe;
    } else if (axis == "-x") {
        cmd.linear.x = -vel_lin;
        cmd.linear.y = -vel_strafe;
    } else if (axis == "+y") {
        cmd.linear.y = vel_lin;
        cmd.linear.x = vel_strafe;
    } else if (axis == "-y") {
        cmd.linear.y = -vel_lin;
        cmd.linear.x = -vel_strafe;
    }
    cmd.angular.z = vel_ang;

    cmd_pub_->publish(cmd);

    // Stop and Exit Condition
    if (std::abs(error_lin) < this->get_parameter("error_tolerance_lin").as_double() &&
        std::abs(error_ang) < this->get_parameter("error_tolerance_ang").as_double() &&
        std::abs(error_strafe) < this->get_parameter("error_tolerance_strafe").as_double())
    {
      RCLCPP_INFO(this->get_logger(), "Alignment and Centering Complete. Stopping.");
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