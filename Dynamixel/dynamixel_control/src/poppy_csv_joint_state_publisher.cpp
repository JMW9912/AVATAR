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
#include <map>

#define CONTROL_FPS 30   // poppy_read_write_node의 CONTROL_FPS와 동일하게 유지

namespace fs = std::filesystem;

class CSVJointStatePublisher : public rclcpp::Node {
public:
    CSVJointStatePublisher()
        : Node("poppy_csv_joint_state_publisher"),
          default_folder_("/home/minwoong/OneDrive/JMW/dynamixel_8DOF/Dynamixel/dynamixel_control/data/csv/poppy"),
          default_file_(),
          stop_current_(false){ 

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
    std::string default_file_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    bool stop_current_;

    std::vector<std::string> listCSVFiles(const std::string &folder_path) {
        std::vector<std::string> csv_files;
        for (const auto &entry : fs::directory_iterator(folder_path)) {
            if (entry.path().extension() == ".csv") {
                csv_files.push_back(entry.path().string());
            }
        }
    
        // 파일 이름 정렬
        std::sort(csv_files.begin(), csv_files.end(), [](const std::string &a, const std::string &b) {
            auto extractNumber = [](const std::string &filename) -> int {
                size_t pos = filename.find_last_of("/\\");
                std::string base_name = (pos == std::string::npos) ? filename : filename.substr(pos + 1);
                size_t num_start = base_name.find_first_of("0123456789");
                if (num_start != std::string::npos) {
                    size_t num_end = base_name.find_first_not_of("0123456789", num_start);
                    return std::stoi(base_name.substr(num_start, num_end - num_start));
                }
                return INT_MAX; // 숫자가 없을 경우 큰 값 반환
            };
    
            return extractNumber(a) < extractNumber(b);
        });
    
        return csv_files;
    }
    

    

    void mainLoop() {
        while (rclcpp::ok()) {
            size_t selected_index = getUserInput();

            if (selected_index < 1 || selected_index > file_list_.size()) {
                RCLCPP_WARN(this->get_logger(), "Invalid index. Using default file: %s", default_file_.c_str());
                processCSVFile(default_file_);
            } else {
                std::string selected_file = file_list_[selected_index - 1];
                RCLCPP_INFO(this->get_logger(), "Selected file: %s", selected_file.c_str());
                processCSVFile(selected_file);
            }
        }
    }

    size_t getUserInput() {
        struct pollfd fd;
        fd.fd = STDIN_FILENO; // 표준 입력 파일 디스크립터
        fd.events = POLLIN;

        RCLCPP_INFO(this->get_logger(), "Available CSV files:");
        for (size_t i = 0; i < file_list_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "[%zu] %s", i + 1, file_list_[i].c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Enter the file index to process (default: 1):");
        size_t selected_index = 0;

        auto start_time = std::chrono::steady_clock::now();
        while (true) {
            RCLCPP_DEBUG(this->get_logger(), "Start time %ld, Now time %ld.",
                std::chrono::duration_cast<std::chrono::seconds>(start_time.time_since_epoch()).count(),
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count());

            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(10)) {
                RCLCPP_INFO(this->get_logger(), "No input detected. Using default file.");
                return 1; // 디폴트 파일
            }

            int ret = poll(&fd, 1, 100); // 100ms 동안 대기
            if (ret > 0) { // 입력이 감지됨
                std::cin >> selected_index;
                return selected_index;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 짧은 대기
        }
    }
    

    void processCSVFile(const std::string &file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", file_path.c_str());
            return;
        }

        std::vector<std::vector<double>> angle_data;
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::stringstream ss(line);
            std::string first_cell;

            // 첫 셀이 숫자가 아니면 헤더 or 주석 행으로 판단하고 조용히 스킵
            if (!std::getline(ss, first_cell, ',')) continue;
            first_cell.erase(std::remove_if(first_cell.begin(), first_cell.end(), ::isspace), first_cell.end());
            double first_val;
            try {
                first_val = std::stod(first_cell);
            } catch (...) {
                RCLCPP_INFO(this->get_logger(), "Header row skipped: %.60s...", line.c_str());
                continue;
            }

            // 나머지 셀 파싱
            std::vector<double> row;
            row.push_back(first_val);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                cell.erase(std::remove_if(cell.begin(), cell.end(), ::isspace), cell.end());
                if (cell.empty()) continue;
                try {
                    row.push_back(std::stod(cell));
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Non-numeric value in data row: '%s'", cell.c_str());
                }
            }
            angle_data.push_back(row);
        }
        file.close();

        RCLCPP_INFO(this->get_logger(), "Loaded %zu frames from: %s", angle_data.size(), file_path.c_str());

        const auto frame_duration = std::chrono::microseconds(1000000 / CONTROL_FPS);
        int frame_idx = 0;

        for (const auto &row : angle_data) {
            auto frame_start = std::chrono::steady_clock::now();

            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();
            // CSV의 조인트 외 나머지(다리 등)는 0으로 채워 초기 자세 유지
            message.name     = allJointNames();
            message.position = buildFullPosition(row);
            publisher_->publish(message);

            // 100프레임마다 진행 상황 출력
            if (frame_idx % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Frame %d / %zu", frame_idx, angle_data.size());
            }
            RCLCPP_DEBUG(this->get_logger(), "Frame %d: published %zu joints", frame_idx, row.size());
            frame_idx++;

            // 발행 처리 시간을 제외한 나머지 sleep (타이밍 보정)
            auto elapsed = std::chrono::steady_clock::now() - frame_start;
            if (elapsed < frame_duration)
                std::this_thread::sleep_for(frame_duration - elapsed);
        }

        RCLCPP_INFO(this->get_logger(), "Finished publishing %d frames from: %s", frame_idx, file_path.c_str());
    }

    // CSV 컬럼 순서와 동일한 15개 제어 조인트
    static const std::vector<std::string>& csvJointNames() {
        static const std::vector<std::string> names = {
            "r_shoulder_y", "r_shoulder_x", "r_arm_z", "r_elbow_y",
            "l_shoulder_y", "l_shoulder_x", "l_arm_z", "l_elbow_y",
            "head_z", "head_y", "bust_x", "bust_y", "abs_z",
            "abs_x", "abs_y"   // 모터 14(인덱스 13), 모터 15(인덱스 14)
        };
        return names;
    }

    // URDF의 모든 비고정 조인트
    // ※ 앞 15개(인덱스 0~14)는 반드시 csvJointNames() 순서와 동일해야 함
    //   → read_write_node가 msg->position[i] 인덱스로 모터 값을 읽기 때문
    static const std::vector<std::string>& allJointNames() {
        static const std::vector<std::string> names = {
            // 인덱스 0~14: read_write_node가 인덱스로 읽는 제어 조인트
            "r_shoulder_y", "r_shoulder_x", "r_arm_z",   "r_elbow_y",
            "l_shoulder_y", "l_shoulder_x", "l_arm_z",   "l_elbow_y",
            "head_z",       "head_y",
            "bust_x",       "bust_y",       "abs_z",
            "abs_x",        "abs_y",        // 모터 14, 15
            // 인덱스 15~24: 비제어 조인트 — 0.0 고정 (시각화용)
            "r_hip_x",  "r_hip_z",  "r_hip_y",  "r_knee_y", "r_ankle_y",
            "l_hip_x",  "l_hip_z",  "l_hip_y",  "l_knee_y", "l_ankle_y",
        };
        return names;
    }

    // CSV 행(degree) → 전체 조인트 position 벡터(radian), 미제어 조인트는 0.0
    std::vector<double> buildFullPosition(const std::vector<double> &csv_row) {
        const auto &csv_names = csvJointNames();
        std::map<std::string, double> controlled;
        for (size_t i = 0; i < csv_names.size() && i < csv_row.size(); ++i)
            controlled[csv_names[i]] = csv_row[i] * M_PI / 180.0;

        std::vector<double> result;
        for (const auto &name : allJointNames())
            result.push_back(controlled.count(name) ? controlled.at(name) : 0.0);
        return result;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSVJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
