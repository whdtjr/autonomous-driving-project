#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class MovePublisher : public rclcpp::Node
{
public:
  MovePublisher()
  : rclcpp::Node("move_publisher"),
    forward_(true),
    switch_period_(std::chrono::seconds(5)),
    last_switch_(std::chrono::steady_clock::now())
  {
    // /cmd_vel 토픽으로 Twist 발행, 큐 사이즈 10
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // network 상황에 따라 잘 publish하도록 10개까지 보관해놓고 전달


    // 0.1초 주기 타이머
    timer_ = this->create_wall_timer(100ms, std::bind(&MovePublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "MovePublisher has been started");
  }

private:
  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_switch_ >= switch_period_) {
      forward_ = !forward_;
      last_switch_ = now;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = forward_ ? 0.2 : -0.2;  // 전진/후진 속도 (m/s)

    pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Moving %s: Linear velocity = %.2f m/s",
                (forward_ ? "forward" : "backward"), msg.linear.x);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool forward_;
  std::chrono::milliseconds switch_period_;
  std::chrono::steady_clock::time_point last_switch_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovePublisher>());
  rclcpp::shutdown();
  return 0;
}
