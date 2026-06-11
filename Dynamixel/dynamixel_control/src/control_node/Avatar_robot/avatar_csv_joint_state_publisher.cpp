#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <poll.h>
#include <climits>
#include <algorithm>

// avatar_read_write_node 의 CONTROL_FPS 와 반드시 동일하게 유지
#define CONTROL_FPS 120

namespace fs = std::filesystem;

class AvatarCSVJointStatePublisher : public rclcpp::Node {
public:
    AvatarCSVJointStatePublisher()
        : Node("avatar_csv_joint_state_publisher"),
          default_folder_("/home/minwoong/OneDrive/JMW/dynamixel_8DOF/Dynamixel/dynamixel_control/data/csv/avatar"),
          stop_current_(false) {

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "Scanning for CSV files in folder: %s", default_folder_.c_str());
        file_list_ = listCSVFiles(default_folder_);
        if (file_list_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No CSV files found in the specified folder. Exiting.");
            rclcpp::shutdown();
            return;
        }

        mainLoop();
    }

private:
    std::string default_folder_;
    std::vector<std::string> file_list_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    bool stop_current_;

    std::vector<std::string> listCSVFiles(const std::string &folder_path) {
        std::vector<std::string> csv_files;
        for (const auto &entry : fs::directory_iterator(folder_path)) {
            if (entry.path().extension() == ".csv") {
                csv_files.push_back(entry.path().string());
            }
        }
        std::sort(csv_files.begin(), csv_files.end(), [](const std::string &a, const std::string &b) {
            auto extractNumber = [](const std::string &filename) -> int {
                size_t pos = filename.find_last_of("/\\");
                std::string base = (pos == std::string::npos) ? filename : filename.substr(pos + 1);
                size_t num_start = base.find_first_of("0123456789");
                if (num_start != std::string::npos) {
                    size_t num_end = base.find_first_not_of("0123456789", num_start);
                    return std::stoi(base.substr(num_start, num_end - num_start));
                }
                return INT_MAX;
            };
            return extractNumber(a) < extractNumber(b);
        });
        return csv_files;
    }

    void mainLoop() {
        while (rclcpp::ok()) {
            size_t selected_index = getUserInput();
            if (selected_index < 1 || selected_index > file_list_.size()) {
                RCLCPP_WARN(this->get_logger(), "Invalid index. Using file [1].");
                processCSVFile(file_list_[0]);
            } else {
                std::string selected_file = file_list_[selected_index - 1];
                RCLCPP_INFO(this->get_logger(), "Selected file: %s", selected_file.c_str());
                processCSVFile(selected_file);
            }
        }
    }

    size_t getUserInput() {
        struct pollfd fd;
        fd.fd = STDIN_FILENO;
        fd.events = POLLIN;

        RCLCPP_INFO(this->get_logger(), "Available CSV files:");
        for (size_t i = 0; i < file_list_.size(); ++i)
            RCLCPP_INFO(this->get_logger(), "[%zu] %s", i + 1, file_list_[i].c_str());
        RCLCPP_INFO(this->get_logger(), "Enter the file index to process (default: 1, timeout 10s):");

        size_t selected_index = 0;
        auto start_time = std::chrono::steady_clock::now();
        while (true) {
            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(10)) {
                RCLCPP_INFO(this->get_logger(), "No input detected. Using default file [1].");
                return 1;
            }
            int ret = poll(&fd, 1, 100);
            if (ret > 0) {
                std::cin >> selected_index;
                return selected_index;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void processCSVFile(const std::string &file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open: %s", file_path.c_str());
            return;
        }

        std::vector<std::vector<double>> angle_data;
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::stringstream ss(line);
            std::string first_cell;
            if (!std::getline(ss, first_cell, ',')) continue;
            first_cell.erase(std::remove_if(first_cell.begin(), first_cell.end(), ::isspace),
                             first_cell.end());
            double first_val;
            try {
                first_val = std::stod(first_cell);
            } catch (...) {
                RCLCPP_INFO(this->get_logger(), "Header row skipped: %.60s...", line.c_str());
                continue;
            }

            std::vector<double> row;
            row.push_back(first_val);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                cell.erase(std::remove_if(cell.begin(), cell.end(), ::isspace), cell.end());
                if (cell.empty()) continue;
                try {
                    row.push_back(std::stod(cell));
                } catch (...) {
                    // 비숫자 셀(frame 열 등) 무시
                }
            }
            angle_data.push_back(row);
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), "Loaded %zu frames from: %s",
                    angle_data.size(), file_path.c_str());

        const auto frame_duration = std::chrono::microseconds(1000000 / CONTROL_FPS);
        int frame_idx = 0;

        for (const auto &row : angle_data) {
            auto frame_start = std::chrono::steady_clock::now();

            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();
            message.name     = jointNames();
            message.position = buildPosition(row);
            publisher_->publish(message);

            if (frame_idx % 100 == 0)
                RCLCPP_INFO(this->get_logger(), "Frame %d / %zu", frame_idx, angle_data.size());
            frame_idx++;

            auto elapsed = std::chrono::steady_clock::now() - frame_start;
            if (elapsed < frame_duration)
                std::this_thread::sleep_for(frame_duration - elapsed);
        }

        RCLCPP_INFO(this->get_logger(), "Finished publishing %d frames from: %s",
                    frame_idx, file_path.c_str());
    }

    // URDF joint1~joint21 (avatar_read_write_node 의 motor ID 순서와 동일)
    static const std::vector<std::string>& jointNames() {
        static const std::vector<std::string> names = {
            "joint1","joint2","joint3","joint4","joint5","joint6","joint7",
            "joint8","joint9","joint10","joint11","joint12","joint13","joint14",
            "joint15","joint16","joint17","joint18","joint19","joint20","joint21"
        };
        return names;
    }

    // CSV 행(degree, theta1~theta21) → radian 벡터
    // CSV 컬럼 순서: theta1...theta21, frame (frame 열은 파싱 시 무시됨)
    std::vector<double> buildPosition(const std::vector<double> &csv_row) {
        const size_t NUM_JOINTS = 21;
        std::vector<double> result(NUM_JOINTS, 0.0);
        for (size_t i = 0; i < NUM_JOINTS && i < csv_row.size(); ++i)
            result[i] = csv_row[i] * M_PI / 180.0;
        return result;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvatarCSVJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
