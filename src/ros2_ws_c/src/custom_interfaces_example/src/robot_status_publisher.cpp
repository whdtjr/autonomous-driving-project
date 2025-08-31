#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/robot_status.hpp>
#include <random>

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher();

private:
    void timer_callback();

    // Publisher and Timer
    rclcpp::Publisher<custom_interfaces::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Random number generators
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> uniform_real_;
    std::uniform_real_distribution<double> theta_dist_;
    std::uniform_real_distribution<double> temp_dist_;
    std::uniform_int_distribution<int> battery_dist_;
};

RobotStatusPublisher::RobotStatusPublisher()
    : Node("robot_status_publisher"),
      rd_(),
      gen_(rd_()),
      uniform_real_(-10.0, 10.0),
      theta_dist_(-3.14, 3.14),
      temp_dist_(0.0, 50.0),
      battery_dist_(0, 100)
{
    // Publisher 생성
    publisher_ = this->create_publisher<custom_interfaces::msg::RobotStatus>(
        "robot_status", 10);
    
    // 타이머 생성 (1Hz로 동작)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotStatusPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Robot Status Publisher has been started");
}

void RobotStatusPublisher::timer_callback()
{
    auto msg = custom_interfaces::msg::RobotStatus();
    
    // 랜덤 값 생성
    msg.x = uniform_real_(gen_);
    msg.y = uniform_real_(gen_);
    msg.theta = theta_dist_(gen_);
    msg.battery_percentage = battery_dist_(gen_);
    msg.temperature = temp_dist_(gen_);
    
    // 메시지 퍼블리시
    publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), 
                "Publishing: x=%.2f, y=%.2f, theta=%.2f, battery=%d%%",
                msg.x, msg.y, msg.theta, msg.battery_percentage);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto robot_status_publisher = std::make_shared<RobotStatusPublisher>();
    rclcpp::spin(robot_status_publisher);
    rclcpp::shutdown();
    return 0;
}