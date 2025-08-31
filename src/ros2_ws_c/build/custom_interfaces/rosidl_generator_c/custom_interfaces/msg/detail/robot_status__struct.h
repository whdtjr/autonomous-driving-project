// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RobotStatus in the package custom_interfaces.
typedef struct custom_interfaces__msg__RobotStatus
{
  double x;
  double y;
  double theta;
  /// %
  int32_t battery_percentage;
  float temperature;
} custom_interfaces__msg__RobotStatus;

// Struct for a sequence of custom_interfaces__msg__RobotStatus.
typedef struct custom_interfaces__msg__RobotStatus__Sequence
{
  custom_interfaces__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
