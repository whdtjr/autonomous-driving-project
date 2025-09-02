#include <chrono>
#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using Empty = std_srvs::srv::Empty;

class EmptyServiceClient : public rclcpp::Node
{
public:
  EmptyServiceClient(const std::string & service_name = "spin")
  : Node("empty_service_client")
  {
    client_ = this->create_client<Empty>(service_name);

    // 서비스가 올라올 때까지 1초 간격으로 대기
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the service."); // service 호출 할 때 호출 가능한지 확인 무한 대기
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    request_ = std::make_shared<Empty::Request>(); // empty에는 넣을 값이 없음 request message는 일단 보내야 send request를 보낼 수 있다.
  }

  void send_request()
  {
    auto far = client_->async_send_request(request_);  // FutureAndRequestId
    future_ = far.future.share();                      // ✅ 권장: 명시적 추출 + share()
    // 필요하다면 요청 id도 저장 가능
    // request_id_ = far.request_id;
  }

  // 편의 함수들
  bool future_ready() const
  {
    return future_.valid() &&
           future_.wait_for(0s) == std::future_status::ready;
  }

  rclcpp::Client<Empty>::SharedFuture & future()
  {
    return future_;
  }

private:
  rclcpp::Client<Empty>::SharedPtr client_;
  Empty::Request::SharedPtr request_;
  rclcpp::Client<Empty>::SharedFuture future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 기본 서비스 이름은 "spin". 필요하면 실행 시 인자로 바꿀 수 있게 해도 좋습니다.
  auto node = std::make_shared<EmptyServiceClient>("spin");
  node->send_request();

  rclcpp::WallRate rate(10);  // 10Hz -> 0.1s 간격 (python의 timeout_sec=0.1과 유사)
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (node->future_ready()) {
      try {
        // Empty 서비스는 응답 필드가 없지만, 예외 체크를 위해 get() 호출
        (void)node->future().get();
        RCLCPP_INFO(node->get_logger(), "service call completed");
      } catch (const std::exception & e) {
        RCLCPP_INFO(node->get_logger(), "service call failed %s", e.what());
      }
      break;
    }

    RCLCPP_INFO(node->get_logger(), "additional work...!");
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
