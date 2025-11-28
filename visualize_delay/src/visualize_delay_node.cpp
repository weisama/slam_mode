#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 添加这个头文件

#include <deque>
#include <cmath>
#include <algorithm>

class VisualizeDelay : public rclcpp::Node
{
public:
    VisualizeDelay() : Node("visualize_delay")
    {
        // 参数声明
        this->declare_parameter<int>("history_size", 500);
        this->declare_parameter<double>("yaw_scale", 2.0);
        
        history_size_ = this->get_parameter("history_size").as_int();
        yaw_scale_ = this->get_parameter("yaw_scale").as_double();

        // 订阅者
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data_raw", 10,
            std::bind(&VisualizeDelay::imuCallback, this, std::placeholders::_1));
            
        vision_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10,
            std::bind(&VisualizeDelay::visionPoseCallback, this, std::placeholders::_1));

        // 发布者 - 用于可视化
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
        
        // 发布yaw值用于调试
        imu_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("imu_yaw", 10);
        vision_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("vision_yaw", 10);

        // 定时器 - 定期更新可视化
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualizeDelay::updateVisualization, this));

        RCLCPP_INFO(this->get_logger(), "VisualizeDelay node initialized");
        RCLCPP_INFO(this->get_logger(), "History size: %d", history_size_);
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double yaw = quaternionToYaw(msg->orientation);
        
        // 存储IMU yaw数据
        ImuData data;
        data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        data.yaw = yaw;
        
        imu_data_.push_back(data);
        
        // 保持数据长度
        if (imu_data_.size() > static_cast<size_t>(history_size_)) {
            imu_data_.pop_front();
        }
        
        // 发布当前yaw值
        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = static_cast<float>(yaw);
        imu_yaw_pub_->publish(yaw_msg);
        
        latest_imu_yaw_ = yaw;
        has_imu_data_ = true;
    }

    void visionPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double yaw = quaternionToYaw(msg->pose.orientation);
        
        // 存储视觉yaw数据
        VisionData data;
        data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        data.yaw = yaw;
        
        vision_data_.push_back(data);
        
        // 保持数据长度
        if (vision_data_.size() > static_cast<size_t>(history_size_)) {
            vision_data_.pop_front();
        }
        
        // 发布当前yaw值
        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = static_cast<float>(yaw);
        vision_yaw_pub_->publish(yaw_msg);
        
        latest_vision_yaw_ = yaw;
        has_vision_data_ = true;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tf_quat;
        // 使用 tf2::fromMsg 的正确方式
        tf2::fromMsg(quat, tf_quat);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        
        return yaw;
    }

    void updateVisualization()
    {
        if (!has_imu_data_ || !has_vision_data_) {
            return;
        }

        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
        
        // 清除之前的标记
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.push_back(clear_marker);

        // 创建坐标轴
        createCoordinateAxes(*marker_array);
        
        // 创建曲线
        createYawCurves(*marker_array);
        
        // 创建图例
        createLegend(*marker_array);
        
        // 发布标记
        marker_pub_->publish(std::move(marker_array));
    }

    void createCoordinateAxes(visualization_msgs::msg::MarkerArray& marker_array)
    {
        // X轴 - 时间轴
        visualization_msgs::msg::Marker x_axis;
        x_axis.header.frame_id = "map";
        x_axis.header.stamp = this->now();
        x_axis.ns = "axes";
        x_axis.id = 0;
        x_axis.type = visualization_msgs::msg::Marker::LINE_STRIP;
        x_axis.action = visualization_msgs::msg::Marker::ADD;
        x_axis.pose.orientation.w = 1.0;
        x_axis.scale.x = 0.02;
        x_axis.color.r = 0.5f;
        x_axis.color.g = 0.5f;
        x_axis.color.b = 0.5f;
        x_axis.color.a = 1.0f;
        
        geometry_msgs::msg::Point p;
        p.x = -5.0; p.y = 0.0; p.z = 0.0;
        x_axis.points.push_back(p);
        p.x = 5.0; p.y = 0.0; p.z = 0.0;
        x_axis.points.push_back(p);
        
        marker_array.markers.push_back(x_axis);

        // Y轴 - yaw轴
        visualization_msgs::msg::Marker y_axis;
        y_axis.header.frame_id = "map";
        y_axis.header.stamp = this->now();
        y_axis.ns = "axes";
        y_axis.id = 1;
        y_axis.type = visualization_msgs::msg::Marker::LINE_STRIP;
        y_axis.action = visualization_msgs::msg::Marker::ADD;
        y_axis.pose.orientation.w = 1.0;
        y_axis.scale.x = 0.02;
        y_axis.color.r = 0.5f;
        y_axis.color.g = 0.5f;
        y_axis.color.b = 0.5f;
        y_axis.color.a = 1.0f;
        
        p.x = 0.0; p.y = -yaw_scale_; p.z = 0.0;
        y_axis.points.push_back(p);
        p.x = 0.0; p.y = yaw_scale_; p.z = 0.0;
        y_axis.points.push_back(p);
        
        marker_array.markers.push_back(y_axis);
    }

    void createYawCurves(visualization_msgs::msg::MarkerArray& marker_array)
    {
        if (imu_data_.empty() || vision_data_.empty()) {
            return;
        }

        // IMU yaw曲线
        visualization_msgs::msg::Marker imu_curve;
        imu_curve.header.frame_id = "map";
        imu_curve.header.stamp = this->now();
        imu_curve.ns = "yaw_curves";
        imu_curve.id = 10;
        imu_curve.type = visualization_msgs::msg::Marker::LINE_STRIP;
        imu_curve.action = visualization_msgs::msg::Marker::ADD;
        imu_curve.pose.orientation.w = 1.0;
        imu_curve.scale.x = 0.03;
        imu_curve.color.r = 0.0f;
        imu_curve.color.g = 1.0f;
        imu_curve.color.b = 0.0f;
        imu_curve.color.a = 1.0f;

        // 视觉yaw曲线
        visualization_msgs::msg::Marker vision_curve;
        vision_curve.header.frame_id = "map";
        vision_curve.header.stamp = this->now();
        vision_curve.ns = "yaw_curves";
        vision_curve.id = 11;
        vision_curve.type = visualization_msgs::msg::Marker::LINE_STRIP;
        vision_curve.action = visualization_msgs::msg::Marker::ADD;
        vision_curve.pose.orientation.w = 1.0;
        vision_curve.scale.x = 0.03;
        vision_curve.color.r = 1.0f;
        vision_curve.color.g = 0.0f;
        vision_curve.color.b = 0.0f;
        vision_curve.color.a = 1.0f;

        // 生成IMU曲线点
        double base_time = imu_data_.back().timestamp;
        for (size_t i = 0; i < imu_data_.size(); ++i) {
            double time_offset = imu_data_[i].timestamp - base_time;
            geometry_msgs::msg::Point p;
            p.x = time_offset;
            p.y = imu_data_[i].yaw;
            p.z = 0.0;
            imu_curve.points.push_back(p);
        }

        // 生成视觉曲线点
        base_time = vision_data_.back().timestamp;
        for (size_t i = 0; i < vision_data_.size(); ++i) {
            double time_offset = vision_data_[i].timestamp - base_time;
            geometry_msgs::msg::Point p;
            p.x = time_offset;
            p.y = vision_data_[i].yaw;
            p.z = 0.0;
            vision_curve.points.push_back(p);
        }

        marker_array.markers.push_back(imu_curve);
        marker_array.markers.push_back(vision_curve);
    }

    void createLegend(visualization_msgs::msg::MarkerArray& marker_array)
    {
        // IMU图例文本
        visualization_msgs::msg::Marker imu_legend;
        imu_legend.header.frame_id = "map";
        imu_legend.header.stamp = this->now();
        imu_legend.ns = "legend";
        imu_legend.id = 20;
        imu_legend.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        imu_legend.action = visualization_msgs::msg::Marker::ADD;
        imu_legend.pose.position.x = -4.5;
        imu_legend.pose.position.y = yaw_scale_ - 0.3;
        imu_legend.pose.position.z = 0.0;
        imu_legend.pose.orientation.w = 1.0;
        imu_legend.scale.z = 0.3;
        imu_legend.color.r = 0.0f;
        imu_legend.color.g = 1.0f;
        imu_legend.color.b = 0.0f;
        imu_legend.color.a = 1.0f;
        imu_legend.text = "IMU Yaw: " + std::to_string(latest_imu_yaw_);
        marker_array.markers.push_back(imu_legend);

        // 视觉图例文本
        visualization_msgs::msg::Marker vision_legend;
        vision_legend.header.frame_id = "map";
        vision_legend.header.stamp = this->now();
        vision_legend.ns = "legend";
        vision_legend.id = 21;
        vision_legend.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        vision_legend.action = visualization_msgs::msg::Marker::ADD;
        vision_legend.pose.position.x = -4.5;
        vision_legend.pose.position.y = yaw_scale_ - 0.6;
        vision_legend.pose.position.z = 0.0;
        vision_legend.pose.orientation.w = 1.0;
        vision_legend.scale.z = 0.3;
        vision_legend.color.r = 1.0f;
        vision_legend.color.g = 0.0f;
        vision_legend.color.b = 0.0f;
        vision_legend.color.a = 1.0f;
        vision_legend.text = "Vision Yaw: " + std::to_string(latest_vision_yaw_);
        marker_array.markers.push_back(vision_legend);
    }

    struct ImuData {
        double timestamp;
        double yaw;
    };

    struct VisionData {
        double timestamp;
        double yaw;
    };

    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vision_yaw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 数据存储
    std::deque<ImuData> imu_data_;
    std::deque<VisionData> vision_data_;
    
    // 当前值
    double latest_imu_yaw_ = 0.0;
    double latest_vision_yaw_ = 0.0;
    
    // 状态标志
    bool has_imu_data_ = false;
    bool has_vision_data_ = false;
    
    // 参数
    int history_size_;
    double yaw_scale_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizeDelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
