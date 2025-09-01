#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

using std::placeholders::_1;

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
  : rclcpp::Node("lidar_subscriber")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_raw", 10, std::bind(&LidarSubscriber::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "LidarSubscriber has been started");
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto & ranges = msg->ranges;
    if (ranges.empty()) {
      RCLCPP_INFO(this->get_logger(), "No scan data");
      return;
    }

    // 전방 180도: 전체의 가운데 절반 (Python 코드와 동일한 슬라이싱 로직)
    const size_t n = ranges.size();
    const size_t start = n / 4;
    const size_t end   = 3 * n / 4;          // [start, end)
    if (end <= start) {
      RCLCPP_INFO(this->get_logger(), "Scan size too small");
      return;
    }
    const size_t center = start + (end - start) / 2;

    // 유효(r>0 && finite) 값 중 최소 탐색
    float best_dist = std::numeric_limits<float>::infinity();
    size_t best_idx = static_cast<size_t>(-1);

    for (size_t i = start; i < end; ++i) {
      float r = ranges[i];
      if (std::isfinite(r) && r > 0.0f) {
        if (r < best_dist) {
          best_dist = r;
          best_idx = i;
        }
      }
    }

    if (best_idx != static_cast<size_t>(-1)) {
      // front 구간 중심을 0으로 두고, index 차이 * angle_increment 로 각도 계산
      int relative_index = static_cast<int>(best_idx) - static_cast<int>(center);
      double angle_rad = static_cast<double>(relative_index) * static_cast<double>(msg->angle_increment);
      double angle_deg = angle_rad * 180.0 / M_PI;

      RCLCPP_INFO(this->get_logger(),
        "Closest obstacle: Distance = %.2f m, Angle = %.2f degrees",
        best_dist, angle_deg);
    } else {
      RCLCPP_INFO(this->get_logger(), "No obstacles detected in the front 180 degrees");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
