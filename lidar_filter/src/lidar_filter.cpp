#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <cmath>

using std::placeholders::_1;

class LidarFilter : public rclcpp::Node
{
public:
  LidarFilter()
  : Node("lidar_filter"), frame_count_(0)
  {
    this->declare_parameter<std::string>("scan_topic", "/scan_raw");
    this->get_parameter("scan_topic", scan_topic_);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&LidarFilter::scanCallback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(),
                "LidarFilter started. Subscribing to: %s",
                scan_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    frame_count_++;

    int original_size = msg->ranges.size();

    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;

    float new_angle_increment =
        (angle_max - angle_min) / (original_size - 1);

    auto output = *msg;
    output.angle_increment = new_angle_increment;

    int output_computed_points =
        static_cast<int>(std::round(
            (output.angle_max - output.angle_min) / output.angle_increment)) + 1;

    // === 每 30 帧打印一次 ===
    if (frame_count_ >= 30)
    {
      RCLCPP_INFO(this->get_logger(),
                  "原始点数：%d | 输出点数：%ld | 计算点数：%d",
                  original_size,
                  output.ranges.size(),
                  output_computed_points);

      frame_count_ = 0;  // 重新计数
    }

    publisher_->publish(output);
  }

  std::string scan_topic_;
  int frame_count_;  // 帧计数器

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>());
  rclcpp::shutdown();
  return 0;
}
