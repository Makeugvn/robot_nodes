#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <vector>

class ScanPublisher : public rclcpp::Node
{
public:
  ScanPublisher() : Node("scan_publisher")
  {
    // Parameter yang bisa diubah via YAML
    this->declare_parameter("num_readings", 90);
    this->declare_parameter("angle_min_deg", -90.0);
    this->declare_parameter("angle_max_deg",  90.0);
    this->declare_parameter("max_range", 4.0);
    this->declare_parameter("min_range", 0.1);
    this->declare_parameter("frame_id", "laser");

    num_readings_ = this->get_parameter("num_readings").as_int();
    max_range_    = this->get_parameter("max_range").as_double();
    min_range_    = this->get_parameter("min_range").as_double();
    frame_id_     = this->get_parameter("frame_id").as_string();

    double ang_min = this->get_parameter("angle_min_deg").as_double();
    double ang_max = this->get_parameter("angle_max_deg").as_double();
    angle_min_ = ang_min * M_PI / 180.0;
    angle_max_ = ang_max * M_PI / 180.0;
    angle_increment_ = (angle_max_ - angle_min_) / (num_readings_ - 1);

    // Subscribe data mentah dari ESP32 (array jarak dalam meter)
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/tfmini_raw", 10,
      std::bind(&ScanPublisher::raw_callback, this, std::placeholders::_1));

    // Publish LaserScan
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(),
      "scan_publisher ready — %d readings, %.1f° to %.1f°",
      num_readings_,
      this->get_parameter("angle_min_deg").as_double(),
      this->get_parameter("angle_max_deg").as_double());
  }

private:
  void raw_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp    = this->now();
    scan.header.frame_id = frame_id_;

    scan.angle_min       = angle_min_;
    scan.angle_max       = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.range_min       = min_range_;
    scan.range_max       = max_range_;
    scan.scan_time       = 1.8f;  // ~1.8 detik per sweep (90 step × 20ms)
    scan.time_increment  = scan.scan_time / num_readings_;

    // Salin data, clamp nilai out-of-range
    scan.ranges.resize(num_readings_);
    for (int i = 0; i < num_readings_ && i < (int)msg->data.size(); i++) {
      float d = msg->data[i];
      scan.ranges[i] = (d >= min_range_ && d <= max_range_) ? d
                       : std::numeric_limits<float>::infinity();
    }

    pub_->publish(scan);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  int    num_readings_;
  double angle_min_, angle_max_, angle_increment_;
  double max_range_, min_range_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPublisher>());
  rclcpp::shutdown();
  return 0;
}