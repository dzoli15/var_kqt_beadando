#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

class CruiseControlNode : public rclcpp::Node {
public:
    CruiseControlNode() : Node("cruise_control_node"), target_speed_(0), current_speed_(0), last_command_(0) {
        command_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/speed_change_command", 10);
        
        current_speed_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/current_speed", 10,
            std::bind(&CruiseControlNode::current_speed_callback, this, std::placeholders::_1));
            
        target_speed_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_speed", 10,
            std::bind(&CruiseControlNode::target_speed_callback, this, std::placeholders::_1));
            
        control_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), // Irányítási ciklus (1 másodperc)
            std::bind(&CruiseControlNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Cruise Control Node elindult");
    }

private:
    void current_speed_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        current_speed_ = msg->data;
    }

    void target_speed_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_speed_ = std::max(0, msg->data);
    }

    void control_loop() {
        auto message = std_msgs::msg::Int32();
        int32_t speed_error = target_speed_ - current_speed_;
        
        if (speed_error == 0) {
            last_command_ = 0;
        } else {
            // Arányos szabályozás egész számokkal: a hiba 30%-a
            int32_t calculated_change = static_cast<int32_t>(round(speed_error * 0.3));

            if (speed_error > 0) { // Gyorsítás érték 1 és 5 között
                last_command_ = std::min(5, std::max(1, calculated_change));
            } else { // Lassítás érték -1 és -5 között
                last_command_ = std::max(-5, std::min(-1, calculated_change));
            }
        }
        
        message.data = last_command_;
        command_publisher_->publish(message);
        auto target_message = std_msgs::msg::Int32();
        target_message.data = target_speed_;
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_speed_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_speed_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    int32_t target_speed_;
    int32_t current_speed_;
    int32_t last_command_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CruiseControlNode>());
    rclcpp::shutdown();
    return 0;
}