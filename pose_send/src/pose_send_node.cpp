#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

class PoseSendNode : public rclcpp::Node
{
public:
    PoseSendNode() : Node("pose_send_node")
    {
        // 参数声明（默认都使能）
        this->declare_parameter<int>("px4_flag", 1);
        this->declare_parameter<int>("uart_flag", 1);

        px4_flag_ = this->get_parameter("px4_flag").as_int();
        uart_flag_ = this->get_parameter("uart_flag").as_int();

        RCLCPP_INFO(this->get_logger(),
                    "px4_flag=%d  uart_flag=%d", px4_flag_, uart_flag_);

        // 声明 use_sim_time
        if (!this->has_parameter("use_sim_time"))
            this->declare_parameter("use_sim_time", false);

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (use_sim_time)
            RCLCPP_INFO(this->get_logger(), "Using simulated time (use_sim_time=true)");

        // 发布 MAVROS
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

        // TF 缓冲
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅 TF
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 50,
            std::bind(&PoseSendNode::tf_callback, this, std::placeholders::_1));

        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", 10,
            std::bind(&PoseSendNode::tf_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "pose_send_node started, waiting for TFs (map → base_link)...");

        // 串口打开（仅当需要）
        if (uart_flag_ == 1)
            open_serial_port();
    }

private:
    //============================================================================
    // 串口初始化
    //============================================================================
    void open_serial_port()
    {
        serial_fd_ = open("/dev/ttyS4", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyS4");
            return;
        }

        struct termios tty;
        tcgetattr(serial_fd_, &tty);

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN] = 0;

        tcsetattr(serial_fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Serial /dev/ttyS4 opened (115200 8N1)");
    }

    //============================================================================
    // 串口发送函数
    //============================================================================
    void send_serial(int16_t x, int16_t y, int16_t z, int16_t yaw)
    {
        if (serial_fd_ < 0 || uart_flag_ == 0)
            return;

        uint8_t buf[10];
        buf[0] = 0xAA;

        buf[1] = (x >> 8) & 0xFF;
        buf[2] = (x)&0xFF;

        buf[3] = (y >> 8) & 0xFF;
        buf[4] = (y)&0xFF;

        buf[5] = (z >> 8) & 0xFF;
        buf[6] = (z)&0xFF;

        buf[7] = (yaw >> 8) & 0xFF;
        buf[8] = (yaw)&0xFF;

        buf[9] = 0x0A;

        write(serial_fd_, buf, 10);
    }

    //============================================================================
    // TF 回调
    //============================================================================
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        (void)msg;

        try
        {
            auto transform_stamped =
                tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0), tf2::durationFromSec(0.1));

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = transform_stamped.header;
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = transform_stamped.transform.translation.x;
            pose_msg.pose.position.y = transform_stamped.transform.translation.y;
            pose_msg.pose.position.z = transform_stamped.transform.translation.z;
            pose_msg.pose.orientation = transform_stamped.transform.rotation;

            // MAVROS 输出（受 px4_flag 控制）
            if (px4_flag_ == 1)
                pose_pub_->publish(pose_msg);

            // 单位转换 m → cm, rad → deg
            int16_t x_cm = static_cast<int16_t>(pose_msg.pose.position.x * 100.0);
            int16_t y_cm = static_cast<int16_t>(pose_msg.pose.position.y * 100.0);
            int16_t z_cm = static_cast<int16_t>(pose_msg.pose.position.z * 100.0);

            tf2::Quaternion q(
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w);

            double roll, pitch, yaw_rad;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_rad);

            int16_t yaw_deg = static_cast<int16_t>(yaw_rad * 180.0 / M_PI);

            // 串口输出（受 uart_flag 控制）
            send_serial(x_cm, y_cm, z_cm, yaw_deg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for TF map→base_link: %s", ex.what());
        }
    }

    //============================================================================
    // 成员变量
    //============================================================================
    int serial_fd_ = -1;
    int px4_flag_ = 1;
    int uart_flag_ = 1;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};


//============================================================================
// main
//============================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSendNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
