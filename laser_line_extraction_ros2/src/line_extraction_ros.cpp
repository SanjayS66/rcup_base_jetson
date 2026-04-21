#include "laser_line_extraction_ros2/line_extraction_ros.hpp"
#include <array>

namespace line_extraction
{
LineExtractionROS::LineExtractionROS() : Node("line_extraction_node"), data_cached_(false)
{
  loadParameters();
  line_publisher_ = this->create_publisher<laser_line_extraction_ros2::msg::LineSegmentList>("line_segments", 1);
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 1, std::bind(&LineExtractionROS::laserScanCallback, this, std::placeholders::_1));
  
  if (pub_markers_)
  {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("line_markers", 1);
  }
}

void LineExtractionROS::loadParameters()
{
  this->declare_parameter<std::string>("frame_id", "front_laser");
  this->declare_parameter<std::string>("scan_topic", "front_scan_filtered");
  this->declare_parameter<bool>("publish_markers", true);
  this->declare_parameter<double>("bearing_std_dev", 1e-3);
  this->declare_parameter<double>("range_std_dev", 0.02);
  this->declare_parameter<double>("least_sq_angle_thresh", 1e-4);
  this->declare_parameter<double>("least_sq_radius_thresh", 1e-4);
  this->declare_parameter<double>("max_line_gap", 0.4);
  this->declare_parameter<double>("min_line_length", 0.5);
  this->declare_parameter<double>("min_range", 0.4);
  this->declare_parameter<double>("max_range", 10000.0);
  this->declare_parameter<double>("min_split_dist", 0.05);
  this->declare_parameter<double>("outlier_dist", 0.05);
  this->declare_parameter<int>("min_line_points", 9);

  frame_id_ = this->get_parameter("frame_id").as_string();
  scan_topic_ = this->get_parameter("scan_topic").as_string();
  pub_markers_ = this->get_parameter("publish_markers").as_bool();

  line_extraction_.setBearingVariance(pow(this->get_parameter("bearing_std_dev").as_double(), 2));
  line_extraction_.setRangeVariance(pow(this->get_parameter("range_std_dev").as_double(), 2));
  line_extraction_.setLeastSqAngleThresh(this->get_parameter("least_sq_angle_thresh").as_double());
  line_extraction_.setLeastSqRadiusThresh(this->get_parameter("least_sq_radius_thresh").as_double());
  line_extraction_.setMaxLineGap(this->get_parameter("max_line_gap").as_double());
  line_extraction_.setMinLineLength(this->get_parameter("min_line_length").as_double());
  line_extraction_.setMinRange(this->get_parameter("min_range").as_double());
  line_extraction_.setMaxRange(this->get_parameter("max_range").as_double());
  line_extraction_.setMinSplitDist(this->get_parameter("min_split_dist").as_double());
  line_extraction_.setOutlierDist(this->get_parameter("outlier_dist").as_double());
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(this->get_parameter("min_line_points").as_int()));
}

void LineExtractionROS::run()
{
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  laser_line_extraction_ros2::msg::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);
  line_publisher_->publish(msg);

  if (pub_markers_)
  {
    visualization_msgs::msg::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_->publish(marker_msg);
  }
}

void LineExtractionROS::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg);
    data_cached_ = true;
  }
  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

void LineExtractionROS::cacheData(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  
  // Use the exact array size to prevent out-of-bounds access
  const std::size_t num_measurements = scan_msg->ranges.size(); 
  
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }
  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
}

void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines, laser_line_extraction_ros2::msg::LineSegmentList &line_list_msg)
{
  for (const auto &line : lines)
  {
    laser_line_extraction_ros2::msg::LineSegment line_msg;
    line_msg.angle = line.getAngle();
    line_msg.radius = line.getRadius();
    auto cov = line.getCovariance();
    std::copy(cov.begin(), cov.end(), line_msg.covariance.begin());
    auto start = line.getStart();
    std::copy(start.begin(), start.end(), line_msg.start.begin());
    auto end = line.getEnd();
    std::copy(end.begin(), end.end(), line_msg.end.begin());
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = this->now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines, visualization_msgs::msg::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0; marker_msg.color.a = 1.0;
  for (const auto &line : lines)
  {
    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = line.getStart()[0]; p_start.y = line.getStart()[1];
    p_end.x = line.getEnd()[0]; p_end.y = line.getEnd()[1];
    marker_msg.points.push_back(p_start);
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = this->now();
}
}