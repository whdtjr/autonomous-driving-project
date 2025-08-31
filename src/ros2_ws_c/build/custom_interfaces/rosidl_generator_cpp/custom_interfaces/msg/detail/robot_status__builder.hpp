// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/robot_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_temperature
{
public:
  explicit Init_RobotStatus_temperature(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RobotStatus temperature(::custom_interfaces::msg::RobotStatus::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_battery_percentage
{
public:
  explicit Init_RobotStatus_battery_percentage(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_temperature battery_percentage(::custom_interfaces::msg::RobotStatus::_battery_percentage_type arg)
  {
    msg_.battery_percentage = std::move(arg);
    return Init_RobotStatus_temperature(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_theta
{
public:
  explicit Init_RobotStatus_theta(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_battery_percentage theta(::custom_interfaces::msg::RobotStatus::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_RobotStatus_battery_percentage(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_y
{
public:
  explicit Init_RobotStatus_y(::custom_interfaces::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_theta y(::custom_interfaces::msg::RobotStatus::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_RobotStatus_theta(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

class Init_RobotStatus_x
{
public:
  Init_RobotStatus_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_y x(::custom_interfaces::msg::RobotStatus::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_RobotStatus_y(msg_);
  }

private:
  ::custom_interfaces::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RobotStatus>()
{
  return custom_interfaces::msg::builder::Init_RobotStatus_x();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
