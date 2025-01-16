#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

class RVizVisualizationNode : public rclcpp::Node {
public:
    RVizVisualizationNode()
        : Node("rviz_visualization_node") {
        // Joint state 토픽 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&RVizVisualizationNode::jointStateCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "RViz Visualization Node initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 수신된 Joint State 데이터를 확인 (디버깅용)
        RCLCPP_INFO(this->get_logger(), "Received Joint States:");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %s: %f", msg->name[i].c_str(), msg->position[i]);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVizVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
