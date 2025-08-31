#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node
{
public:
  TimerNode() : rclcpp::Node("timer_node"), count_(0)
  {
    // 1초마다 콜백 실행
    timer_ = this->create_wall_timer(1s, std::bind(&TimerNode::on_timer, this));
  }

private:
  void on_timer()
  {
    ++count_;
    RCLCPP_INFO(this->get_logger(), "Hello ROS2! Count: %d", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerNode>());
  rclcpp::shutdown();
  return 0;
}
