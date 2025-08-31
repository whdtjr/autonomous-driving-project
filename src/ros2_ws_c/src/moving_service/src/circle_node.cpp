#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class CircleService : public rclcpp::Node
{
public:
  CircleService()
  : Node("circle_service")
  {
    // 서비스 생성
    srv_ = this->create_service<std_srvs::srv::Empty>(
      "circle",
      std::bind(&CircleService::circle_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // 퍼블리셔 생성
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Circle service has been started");
  }

private:
  void circle_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    RCLCPP_INFO(this->get_logger(), "Drawing a circle for 10 seconds...");

    auto start = this->now();

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 1.0;   // 전진 속도
    twist.angular.z = 1.5;  // 회전 속도

    rclcpp::Rate rate(10); // 10 Hz (0.1초 마다 퍼블리시)

    while ((this->now() - start).seconds() < 10.0) {
      publisher_->publish(twist);
      rate.sleep();
    }

    // 정지 메시지 발행
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    publisher_->publish(twist);

    double elapsed = (this->now() - start).seconds();
    RCLCPP_INFO(this->get_logger(), "Circle completed in %.2f seconds", elapsed);
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CircleService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
