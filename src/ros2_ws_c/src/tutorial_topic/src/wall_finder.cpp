#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>

class ContinuousWallFinder : public rclcpp::Node
{
public:
    ContinuousWallFinder();

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();

    // Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state
    std::string state_;
    double front_distance_;
};

ContinuousWallFinder::ContinuousWallFinder()
    : Node("continuous_wall_finder"), 
      state_("FIND_WALL"), 
      front_distance_(std::numeric_limits<double>::infinity())
{
    // Subscriber 생성
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_raw", 10,
        std::bind(&ContinuousWallFinder::lidar_callback, this, std::placeholders::_1));
    
    // Publisher 생성
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // 타이머 생성 (10Hz로 동작)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ContinuousWallFinder::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Continuous Wall Finder node has been started");
}

void ContinuousWallFinder::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 전방 30도(-15도에서 15도까지) 범위의 데이터 처리
    int center_index = msg->ranges.size() / 2;
    int start_index = center_index - 15;
    int end_index = center_index + 15;
    
    // 인덱스 범위 확인
    start_index = std::max(0, start_index);
    end_index = std::min(static_cast<int>(msg->ranges.size()), end_index);
    
    std::vector<float> front_ranges(msg->ranges.begin() + start_index, 
                                   msg->ranges.begin() + end_index);
    
    // 무한대 값과 0값을 제외한 유효한 거리 값 찾기
    std::vector<float> valid_ranges;
    for (const auto& range : front_ranges) {
        if (std::isfinite(range) && range > 0.0f) {
            valid_ranges.push_back(range);
        }
    }
    
    if (!valid_ranges.empty()) {
        front_distance_ = *std::min_element(valid_ranges.begin(), valid_ranges.end());
    } else {
        front_distance_ = std::numeric_limits<double>::infinity();
    }
}

void ContinuousWallFinder::timer_callback()
{
    auto msg = geometry_msgs::msg::Twist();
    
    if (state_ == "FIND_WALL") {
        if (front_distance_ > 4.0) {
            // 좌회전
            msg.linear.x = 0.7;
            msg.angular.z = 0.5;
        } else {
            state_ = "MOVE_TO_WALL";
            RCLCPP_INFO(this->get_logger(), "Wall found. Moving towards it.");
        }
    }
    else if (state_ == "MOVE_TO_WALL") {
        if (front_distance_ > 0.5) {
            msg.linear.x = 0.4;  // 천천히 전진
        } else {
            state_ = "FIND_NEXT_WALL";
            RCLCPP_INFO(this->get_logger(), "Reached wall. Finding next wall.");
        }
    }
    else if (state_ == "FIND_NEXT_WALL") {
        msg.angular.z = 0.7;  // 좌회전
        if (front_distance_ > 4.0) {
            state_ = "FIND_WALL";
            RCLCPP_INFO(this->get_logger(), "Found open space. Searching for next wall.");
        }
    }
    
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "State: %s, Front distance: %.2fm", 
                state_.c_str(), front_distance_);
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto continuous_wall_finder = std::make_shared<ContinuousWallFinder>();
    rclcpp::spin(continuous_wall_finder);
    rclcpp::shutdown();
    return 0;
}
