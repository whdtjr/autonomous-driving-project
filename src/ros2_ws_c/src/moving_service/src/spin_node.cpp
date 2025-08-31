#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

class SpinService : public rclcpp::Node
{
public:
  SpinService()
  : Node("spin_service")
  {
    // 서비스 생성
    srv_ = this->create_service<std_srvs::srv::Empty>(
      "spin",
      std::bind(&SpinService::spin_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // 퍼블리셔 생성
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Spin service has been started");
  }

private:
  void spin_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "Spinning for 5 seconds...");

    auto start = this->now();

    geometry_msgs::msg::Twist twist;
    twist.angular.z = 1.5;  // 회전 속도 (rad/s)

    rclcpp::Rate rate(10); // 10 Hz, 0.1초마다 발행

    while ((this->now() - start).seconds() < 5.0) {
      publisher_->publish(twist);
      rate.sleep();
    }

    // 정지
    twist.angular.z = 0.0;
    publisher_->publish(twist);

    double elapsed = (this->now() - start).seconds();
    RCLCPP_INFO(this->get_logger(), "Spin completed in %.2f seconds", elapsed);
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpinService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
