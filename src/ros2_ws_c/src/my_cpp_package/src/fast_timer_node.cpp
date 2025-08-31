#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class FastTimerNode : public rclcpp::Node
{
public:
  FastTimerNode() : rclcpp::Node("fast_timer_node"), count_(0)
  {
    // 0.1초(100ms)마다 콜백
    timer_ = this->create_wall_timer(100ms, std::bind(&FastTimerNode::on_timer, this));
    // 또는: this->create_wall_timer(std::chrono::milliseconds(100), ...);
  }

private:
  void on_timer()
  {
    ++count_;
    RCLCPP_INFO(this->get_logger(), "Fast Hello ROS2! Count: %d", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastTimerNode>());
  rclcpp::shutdown();
  return 0;
}
