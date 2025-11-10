#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PoseSendNode : public rclcpp::Node
{
public:
    PoseSendNode() : Node("pose_send_node")
    {
        // ✅ 检查是否已声明 use_sim_time（避免重复声明异常）
        if (!this->has_parameter("use_sim_time"))
        {
            this->declare_parameter("use_sim_time", false);
        }

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (use_sim_time)
        {
            RCLCPP_INFO(this->get_logger(), "Using simulated time (use_sim_time=true)");
        }

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

        // 增加 TF 缓存时长以避免 extrapolation
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅 /tf 和 /tf_static，用来触发 lookupTransform
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 50,
            std::bind(&PoseSendNode::tf_callback, this, std::placeholders::_1));
        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", 10,
            std::bind(&PoseSendNode::tf_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "pose_send_node started, waiting for TFs (map→odom→base_link)...");
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        (void)msg; // 不直接用 TF，只触发 TF 缓存更新

        try
        {
            // 使用最新可用 TF，等待 0.1 秒以避免时间不同步问题
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0), tf2::durationFromSec(0.1));

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = transform_stamped.header.stamp;
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = transform_stamped.transform.translation.x;
            pose_msg.pose.position.y = transform_stamped.transform.translation.y;
            pose_msg.pose.position.z = transform_stamped.transform.translation.z;
            pose_msg.pose.orientation = transform_stamped.transform.rotation;

            pose_pub_->publish(pose_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published pose: x=%.2f y=%.2f",
                         pose_msg.pose.position.x, pose_msg.pose.position.y);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for TF map->base_link: %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSendNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
