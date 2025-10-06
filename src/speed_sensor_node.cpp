#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

class VehicleModelNode : public rclcpp::Node {
public:
    VehicleModelNode() : Node("speed_sensor_node"), current_speed_(0), speed_change_command_(0), target_speed_(0) {
        speed_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/current_speed", 10);
        
        // Parancs fogadása
        command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/speed_change_command", 10,
            std::bind(&VehicleModelNode::command_callback, this, std::placeholders::_1));

        // Célsebesség fogadása a logoláshoz. Feltételezve, hogy egy másik node publikálja a /target_speed-et
        target_speed_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/target_speed", 10,
            std::bind(&VehicleModelNode::target_speed_callback, this, std::placeholders::_1));


        // A fizikai modell és a logolás 500ms-onként (0.5 másodpercenként) frissül
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&VehicleModelNode::physics_update_and_log_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Vehicle Model Node elindult. Frissítési ciklus: 0.5s.");
    }

private:
    void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        speed_change_command_ = msg->data;
    }

    void target_speed_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        target_speed_ = msg->data;
    }

    void physics_update_and_log_callback() {
        int32_t speed_change = speed_change_command_;
        current_speed_ += speed_change;

        if (current_speed_ < 0) {
            current_speed_ = 0;
        }

        // Aktuális sebesség publikálása
        auto message = std_msgs::msg::Int32();
        message.data = current_speed_;
        speed_publisher_->publish(message);
        
        // Logolás
        RCLCPP_INFO(this->get_logger(), 
            "[cruise control] Aktuális: %d km/h | Tempomat: %d km/h | Változás: %+d km/h",
            current_speed_, target_speed_, speed_change);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr speed_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_speed_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int32_t current_speed_;
    int32_t speed_change_command_;
    int32_t target_speed_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleModelNode>());
    rclcpp::shutdown();
    return 0;
}