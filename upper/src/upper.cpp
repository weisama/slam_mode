#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <filesystem>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

class TcpSender : public rclcpp::Node
{
public:
    TcpSender() : Node("tcp_sender"), listen_sock_(-1)
    {
        package_share_dir_ = ament_index_cpp::get_package_share_directory("upper");

        this->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
        this->declare_parameter<int>("tcp_port", 6666);

        tcp_ip_ = this->get_parameter("tcp_ip").as_string();
        tcp_port_ = this->get_parameter("tcp_port").as_int();

        startTcpServer();

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&TcpSender::tfCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TcpSender::scanCallback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&TcpSender::mapCallback, this, std::placeholders::_1));

        tcp_accept_timer_ = this->create_wall_timer(200ms, std::bind(&TcpSender::acceptPendingClients, this));
        tcp_recv_timer_ = this->create_wall_timer(100ms, std::bind(&TcpSender::checkTcpRecv, this));
        rate_timer_ = this->create_wall_timer(10s, std::bind(&TcpSender::logRates, this));
    }

    ~TcpSender()
    {
        if (listen_sock_ != -1) close(listen_sock_);
        for (int c : clients_) close(c);
    }

private:
    // ======== TCP Server ========
    void startTcpServer()
    {
        listen_sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "socket创建失败: %s", strerror(errno));
            return;
        }

        int opt = 1;
        setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr {};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(tcp_port_);
        addr.sin_addr.s_addr = inet_addr(tcp_ip_.c_str());

        if (bind(listen_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "端口被占用，Bind failed: %s", strerror(errno));
            return;
        }

        listen(listen_sock_, 5);
        fcntl(listen_sock_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "TCP 正在监听 %s:%d", tcp_ip_.c_str(), tcp_port_);
    }

    void acceptPendingClients()
    {
        sockaddr_in caddr {};
        socklen_t clen = sizeof(caddr);
        int client_fd = accept(listen_sock_, (struct sockaddr *)&caddr, &clen);
        if (client_fd < 0) return;

        fcntl(client_fd, F_SETFL, O_NONBLOCK);
        clients_.insert(client_fd);
        RCLCPP_INFO(this->get_logger(), "新的TCP上位机连接成功, fd=%d", client_fd);
    }

    // ======== TCP 数据接收与解析 ========
    void checkTcpRecv()
    {
        uint8_t buf[256];
        for (auto it = clients_.begin(); it != clients_.end();)
        {
            ssize_t n = recv(*it, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                ++it;
                continue;
            }

            bytes_recv_tcp_ += n;

            std::ostringstream oss;
            oss << "Recv (" << n << " bytes): ";
            for (ssize_t i = 0; i < n; ++i)
                oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                    << static_cast<int>(buf[i]) << " ";
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

            // === 通信测试命令: AA 00 01 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x01 && buf[i + 3] == 0x0A)
                {
                    uint8_t reply[4] = {0xAA, 0x00, 0x01, 0x0A};
                    send(*it, reply, sizeof(reply), 0);
                    RCLCPP_INFO(this->get_logger(), "收到测试指令，回发 AA 00 01 0A");
                }
            }

            // === 保存地图命令: AA 00 02 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x02 && buf[i + 3] == 0x0A)
                {
                    handleMapSaveCommand();
                    RCLCPP_INFO(this->get_logger(), "收到保存地图指令 AA 00 02 0A");
                }
            }

            // === 发送参数命令: AA 00 03 0A ===
            for (ssize_t i = 0; i < n - 3; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x00 &&
                    buf[i + 2] == 0x03 && buf[i + 3] == 0x0A)
                {
                    handleSendParamsCommand();
                    RCLCPP_INFO(this->get_logger(), "收到读取参数指令 AA 00 03 0A");
                }
            }

            // === 参数修改命令: AA 10 param_id val_high val_low 0A ===
            for (ssize_t i = 0; i < n - 5; ++i)
            {
                if (buf[i] == 0xAA && buf[i + 1] == 0x10 && buf[i + 5] == 0x0A)
                {
                    uint8_t param_id = buf[i + 2];
                    uint16_t value = (buf[i + 3] << 8) | buf[i + 4];
                    handleParamCommand(param_id, value);
                    RCLCPP_INFO(this->get_logger(), "收到参数修改: id=%d value=%d", param_id, value);
                }
            }

            ++it;
        }
    }

    // ======== 发送参数到上位机 ========
    void handleSendParamsCommand()
    {
        std::string file_path = package_share_dir_ + "/config/params.yaml";
        YAML::Node config;
        try {
            config = YAML::LoadFile(file_path);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开 %s: %s", file_path.c_str(), e.what());
            return;
        }

        std::string lidar_name = config["lidar_name"] ? config["lidar_name"].as<std::string>() : "N10";
        float x = config["x"] ? config["x"].as<float>() : 0.0f;
        float y = config["y"] ? config["y"].as<float>() : 0.0f;
        float z = config["z"] ? config["z"].as<float>() : 0.0f;
        float roll = config["roll"] ? config["roll"].as<float>() : 0.0f;
        float pitch = config["pitch"] ? config["pitch"].as<float>() : 0.0f;
        float yaw = config["yaw"] ? config["yaw"].as<float>() : 0.0f;
        std::string mode = config["mode"] ? config["mode"].as<std::string>() : "mapping";
        int px4_flag  = config["px4_flag"]  ? config["px4_flag"].as<int>()  : 0;
        int uart_flag = config["uart_flag"] ? config["uart_flag"].as<int>() : 0;

        sendParamToClients(0x00, (lidar_name == "N10") ? 0 : 1);
        sendParamToClients(0x01, static_cast<uint16_t>(x));
        sendParamToClients(0x02, static_cast<uint16_t>(y));
        sendParamToClients(0x03, static_cast<uint16_t>(z));
        sendParamToClients(0x04, static_cast<uint16_t>(roll));
        sendParamToClients(0x05, static_cast<uint16_t>(pitch));
        sendParamToClients(0x06, static_cast<uint16_t>(yaw));
        sendParamToClients(0x10, (mode == "mapping") ? 0 : 1);
        sendParamToClients(0x11, static_cast<uint16_t>(px4_flag));
        sendParamToClients(0x12, static_cast<uint16_t>(uart_flag));
        
        RCLCPP_INFO(this->get_logger(), "参数已从 YAML 文件读取并发送");
    }

    void sendParamToClients(uint8_t param_id, uint16_t value)
    {
        std::vector<uint8_t> packet = {0xAA, 0x10, param_id,
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF),
            0x0A};
        sendPacket(packet);
    }

    // ======== 修改 params.yaml ========
    void handleParamCommand(uint8_t param_id, uint16_t value)
    {
        namespace fs = std::filesystem;
        const std::string file_path = fs::path(package_share_dir_) / "config" / "params.yaml";

        YAML::Node cfg;
        try {
            cfg = YAML::LoadFile(file_path);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "加载 YAML 失败: %s", e.what());
            return;
        }

        /* 更新节点 */
        switch (param_id) {
            case 0x00: cfg["lidar_name"] = (value == 0 ? "N10" : "N10_P"); break;
            case 0x01: cfg["x"]         = static_cast<int>(value); break;
            case 0x02: cfg["y"]         = static_cast<int>(value); break;
            case 0x03: cfg["z"]         = static_cast<int>(value); break;
            case 0x04: cfg["roll"]      = static_cast<int>(value); break;
            case 0x05: cfg["pitch"]     = static_cast<int>(value); break;
            case 0x06: cfg["yaw"]       = static_cast<int>(value); break;
            case 0x10: cfg["mode"]      = (value == 0 ? "mapping" : "localization"); break;
            case 0x11: cfg["px4_flag"]  = static_cast<int>(value); break;
            case 0x12: cfg["uart_flag"] = static_cast<int>(value); break;

            default:
                RCLCPP_WARN(get_logger(), "未知 param_id: %u", param_id);
                return;
        }

        /* 写回文件 */
        try {
            std::ofstream fout(file_path);
            fout << "# 雷达型号选择: N10, N10_P\n"
                        << "lidar_name: " << cfg["lidar_name"].as<std::string>() << "\n\n"
                        << "# 定义 base_link -> laser_link 的固定 TF 变换，单位cm和度\n"
                        << "x: "      << cfg["x"].as<int>()    << "\n"
                        << "y: "      << cfg["y"].as<int>()    << "\n"
                        << "z: "      << cfg["z"].as<int>()    << "\n"
                        << "roll: "   << cfg["roll"].as<int>() << "\n"
                        << "pitch: "  << cfg["pitch"].as<int>()<< "\n"
                        << "yaw: "    << cfg["yaw"].as<int>()  << "\n\n"
                        << "# SLAM_TOOLBOX建图or定位模式，mapping/localization\n"
                        << "mode: "   << cfg["mode"].as<std::string>() << "\n\n"
                        << "# 是否使能飞控输出坐标\n"
                        << "px4_flag: " << cfg["px4_flag"].as<int>() << "\n\n"
                        << "# 是否使能串口输出坐标\n"
                        << "uart_flag: " << cfg["uart_flag"].as<int>() << "\n";
            
            RCLCPP_INFO(get_logger(), "已更新并保存 %s", file_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "保存 YAML 失败: %s", e.what());
        }
    }

    // ======== 保存地图为YAML和PGM格式 ========
    void handleMapSaveCommand()
    {
        if (!last_map_)
        {
            RCLCPP_WARN(this->get_logger(), "没有/map话题数据");
            return;
        }

        std::string dir_path = package_share_dir_ + "/map/";
        if (!fs::exists(dir_path))
            fs::create_directories(dir_path);

        std::string yaml_path = dir_path + "my_map.yaml";
        std::string pgm_path  = dir_path + "my_map.pgm";

        if (!saveMapAsPGM(pgm_path) || !saveMapAsYAML(yaml_path, pgm_path))
            return;

        RCLCPP_INFO(this->get_logger(), "Map 成功保存在 %s 和 %s", yaml_path.c_str(), pgm_path.c_str());
    }

    bool saveMapAsPGM(const std::string& file_path)
    {
        std::ofstream ofs(file_path, std::ios::binary);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开 PGM 文件: %s", file_path.c_str());
            return false;
        }

        const auto& map = last_map_;
        int width = map->info.width;
        int height = map->info.height;

        ofs << "P5\n" << width << " " << height << "\n255\n";
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                int8_t value = map->data[index];
                uint8_t pgm_value;
                if (value == -1) pgm_value = 205;
                else pgm_value = static_cast<uint8_t>((100 - value) * 254 / 100);
                ofs << pgm_value;
            }
        }
        return true;
    }

    bool saveMapAsYAML(const std::string& yaml_path, const std::string& pgm_path)
    {
        std::ofstream ofs(yaml_path);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开 YAML 文件: %s", yaml_path.c_str());
            return false;
        }

        const auto& map = last_map_;
        ofs << "image: my_map.pgm\n"
            << "resolution: " << map->info.resolution << "\n"
            << "origin: [" << map->info.origin.position.x << ", "
                          << map->info.origin.position.y << ", "
                          << map->info.origin.position.z << "]\n"
            << "negate: 0\n"
            << "occupied_thresh: 0.65\n"
            << "free_thresh: 0.196\n";
        return true;
    }

    // ======== ROS2 回调 ========
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        if (msg->transforms.empty()) return;
        const auto &t = msg->transforms[0];
        json j;
        j["topic"] = "tf";
        j["frame_id"] = t.header.frame_id;
        j["child_frame_id"] = t.child_frame_id;
        j["x"] = t.transform.translation.x;
        j["y"] = t.transform.translation.y;
        j["z"] = t.transform.translation.z;
        j["qx"] = t.transform.rotation.x;
        j["qy"] = t.transform.rotation.y;
        j["qz"] = t.transform.rotation.z;
        j["qw"] = t.transform.rotation.w;
        sendJsonPacket(0x01, j.dump());
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.empty()) return;
        json j;
        j["topic"] = "scan";
        j["angle_min"] = msg->angle_min;
        j["angle_max"] = msg->angle_max;
        j["angle_increment"] = msg->angle_increment;
        j["range_count"] = msg->ranges.size();
        j["ranges"] = msg->ranges;
        sendJsonPacket(0x02, j.dump());
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        last_map_ = msg; // 保存最新地图

        json j;
        j["topic"] = "map";
        j["width"] = msg->info.width;
        j["height"] = msg->info.height;
        j["resolution"] = msg->info.resolution;
        j["origin_x"] = msg->info.origin.position.x;
        j["origin_y"] = msg->info.origin.position.y;

        // RLE 压缩
        std::vector<std::pair<int, int>> rle;
        const auto &data = msg->data;
        if (!data.empty()) {
            int current = data[0];
            int count = 1;
            for (size_t i = 1; i < data.size(); ++i) {
                if (data[i] == current && count < 255)
                    count++;
                else {
                    rle.emplace_back(current, count);
                    current = data[i];
                    count = 1;
                }
            }
            rle.emplace_back(current, count);
        }

        json rle_json = json::array();
        for (auto &p : rle)
            rle_json.push_back({p.first, p.second});
        j["rle"] = rle_json;

        sendJsonPacket(0x03, j.dump());
    }

    // ======== JSON 发送 ========
    void sendJsonPacket(uint8_t topic_id, const std::string &json_str)
    {
        std::vector<uint8_t> packet;
        packet.push_back(0xAA);
        packet.push_back(topic_id);
        packet.insert(packet.end(), json_str.begin(), json_str.end());
        packet.push_back(0x0A);
        sendPacket(packet);
    }

    // ======== 实际发送 ========
    void sendPacket(const std::vector<uint8_t> &packet)
    {
        for (auto it = clients_.begin(); it != clients_.end();)
        {
            ssize_t n = send(*it, packet.data(), packet.size(), 0);
            if (n <= 0)
            {
                close(*it);
                it = clients_.erase(it);
            }
            else
            {
                bytes_sent_tcp_ += n;
                ++it;
            }
        }
    }

    void logRates()
    {
        RCLCPP_INFO(this->get_logger(), "TCP发送累计字节: %lu, 接收累计字节: %lu",
                    bytes_sent_tcp_, bytes_recv_tcp_);
    }

    // ========== 成员变量 ==========
    std::string tcp_ip_;
    int tcp_port_;
    int listen_sock_;
    std::set<int> clients_;
    std::mutex mtx_;

    std::string package_share_dir_;

    size_t bytes_sent_tcp_ = 0;
    size_t bytes_recv_tcp_ = 0;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;

    rclcpp::TimerBase::SharedPtr tcp_accept_timer_;
    rclcpp::TimerBase::SharedPtr tcp_recv_timer_;
    rclcpp::TimerBase::SharedPtr rate_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
