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
  : Node("lidar_filter")
  {
    // 参数：输入话题与目标点数
    this->declare_parameter<std::string>("scan_topic", "/scan_raw");
    this->declare_parameter<int>("target_size", 450);

    this->get_parameter("scan_topic", scan_topic_);
    this->get_parameter("target_size", target_size_);

    // 订阅激光
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10, std::bind(&LidarFilter::scanCallback, this, _1));

    // 发布到 /scan（SLAM 使用）
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(this->get_logger(),
                "LidarFilter started. Subscribing to: %s, output size: %d",
                scan_topic_.c_str(), target_size_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int original_size = msg->ranges.size();

    // === 固定点数输出 ===
    std::vector<float> filtered_ranges = msg->ranges;

    if (original_size > target_size_) {
      filtered_ranges.resize(target_size_);
    } else if (original_size < target_size_) {
      filtered_ranges.resize(target_size_, msg->ranges.back());
    }

    // === 固定角度增量（方案二）===
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float fixed_angle_increment = (angle_max - angle_min) / (target_size_ - 1);

    // === 构造输出消息 ===
    auto output = *msg;
    output.ranges = filtered_ranges;
    output.angle_increment = fixed_angle_increment;
    output.scan_time = msg->scan_time;
    output.time_increment = output.scan_time / target_size_;

    // === 计算输出话题点数（angle_max、angle_min、angle_increment） ===
    int output_computed_points =
        static_cast<int>(std::round((output.angle_max - output.angle_min) /
                                     output.angle_increment)) + 1;

    // === 每帧打印信息 ===
    RCLCPP_INFO(this->get_logger(),
                "过滤前点云数：%d | 过滤后点云数：%ld | 计算点云数：%d",
                original_size,
                filtered_ranges.size(),
                output_computed_points);

    publisher_->publish(output);
  }

  std::string scan_topic_;
  int target_size_;

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
