#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <termios.h>
#include <unistd.h>

namespace fs = std::filesystem;

class TorqueCheck : public rclcpp::Node {
public:
    TorqueCheck()
        : Node("torque_check"),
          current_index_(0) {

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // std::string csv_file = "/home/minwoong/OneDrive/JMW/dynamixel_8DOF/Dynamixel/dynamixel_control/data/csv/torquecheck2.csv"; // torquecheck2.csv Update this path
        std::string csv_file = "/home/jmw/ROS2/AVATAR/Dynamixel/dynamixel_control/data/csv/torquecheck2.csv";
        loadCSV(csv_file);

        RCLCPP_INFO(this->get_logger(), "Waiting for keyboard input to publish joint states...");
        keyboardControl();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::vector<std::vector<double>> angle_data_;
    size_t current_index_;

    void loadCSV(const std::string &file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", file_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream stream(line);
            std::vector<double> row_data;
            std::string value;
            for (size_t col = 0; col < 21 && std::getline(stream, value, ','); ++col) {
                try {
                    row_data.push_back(std::stod(value));
                } catch (const std::invalid_argument &e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid data in file. Skipping row.");
                }
            }
            angle_data_.push_back(row_data);
        }
        file.close();
    }

    void keyboardControl() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char c;
        while (rclcpp::ok()) {
            c = getchar();
            if (c == '\x1b') { // Escape sequence for arrow keys
                getchar(); // Skip the '[' character
                switch(getchar()) {
                    case 'C': // Right arrow key
                        if (current_index_ < angle_data_.size() - 1) {
                            current_index_++;
                            publishJointState(current_index_);
                        }
                        break;
                    case 'D': // Left arrow key
                        if (current_index_ > 0) {
                            current_index_--;
                            publishJointState(current_index_);
                        }
                        break;
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void publishJointState(size_t index) {
        if (index < angle_data_.size()) {
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();
            message.name = generateJointNames(angle_data_[index].size());
            message.position = degreesToRadians(angle_data_[index]);

            RCLCPP_INFO(this->get_logger(), "Publishing joint states for index %zu", index);
            publisher_->publish(message);
        }
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
    rclcpp::spin(std::make_shared<TorqueCheck>());
    rclcpp::shutdown();
    return 0;
}
