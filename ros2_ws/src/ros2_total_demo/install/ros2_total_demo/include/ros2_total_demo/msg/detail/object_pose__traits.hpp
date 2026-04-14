// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_total_demo:msg/ObjectPose.idl
// generated code does not contain a copyright notice

#ifndef ROS2_TOTAL_DEMO__MSG__DETAIL__OBJECT_POSE__TRAITS_HPP_
#define ROS2_TOTAL_DEMO__MSG__DETAIL__OBJECT_POSE__TRAITS_HPP_

#include "ros2_total_demo/msg/detail/object_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_total_demo::msg::ObjectPose>()
{
  return "ros2_total_demo::msg::ObjectPose";
}

template<>
inline const char * name<ros2_total_demo::msg::ObjectPose>()
{
  return "ros2_total_demo/msg/ObjectPose";
}

template<>
struct has_fixed_size<ros2_total_demo::msg::ObjectPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_total_demo::msg::ObjectPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_total_demo::msg::ObjectPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2_TOTAL_DEMO__MSG__DETAIL__OBJECT_POSE__TRAITS_HPP_
