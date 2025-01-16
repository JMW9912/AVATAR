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

namespace fs = std::filesystem;

class CSVJointStatePublisher : public rclcpp::Node {
public:
    CSVJointStatePublisher()
        : Node("csv_joint_state_publisher"),
          default_folder_("/home/minwoong/OneDrive/JMW/dynamixel_8DOF/Dynamixel/dynamixel_control/data/csv"),
          default_file_(),
          stop_current_(false) {

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "Scanning for CSV files in folder: %s", default_folder_.c_str());
        file_list_ = listCSVFiles(default_folder_);
        if (file_list_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No CSV files found in the specified folder. Exiting.");
            rclcpp::shutdown();
            return;
        }

        default_file_ = file_list_[0]; // 기본 CSV 파일은 첫 번째 파일로 설정
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

            if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(3)) {
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
            std::istringstream stream(line);
            std::vector<double> row_data;
            std::string value;
            // while (std::getline(stream, value, ',')) {
            //     try {
            //         row_data.push_back(std::stod(value));
            //     } catch (const std::invalid_argument &e) {
            //         RCLCPP_WARN(this->get_logger(), "Invalid data in file. Skipping row.");
            //     }
            // }
            for (size_t col = 0; col < 21 && std::getline(stream, value, ','); ++col) {
                try {
                    row_data.push_back(std::stod(value));
                } catch (const std::invalid_argument &e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid data in file. Skipping row.");
                }
            }
            angle_data.push_back(row_data);
        }
        file.close();

        for (const auto &row : angle_data) {
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();
            message.name = generateJointNames(row.size());
            message.position = degreesToRadians(row);

            RCLCPP_INFO(this->get_logger(), "Publishing joint states:");
            for (size_t i = 0; i < row.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.2f degrees (%.2f rad)", i + 1, row[i], message.position[i]);
            }

            publisher_->publish(message);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 50ms 간격으로 퍼블리시
        }

        RCLCPP_INFO(this->get_logger(), "Finished publishing file: %s", file_path.c_str());
    }

    std::vector<std::string> generateJointNames(size_t count) {
        std::vector<std::string> joint_names;
        for (size_t i = 1; i <= count; ++i) {
            joint_names.push_back("joint" + std::to_string(i));
        }
        return joint_names;
    }

    std::vector<double> degreesToRadians(const std::vector<double> &degrees) {
        std::vector<double> radians;
        for (double deg : degrees) {
            radians.push_back(deg * M_PI / 180.0);
        }
        return radians;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CSVJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
