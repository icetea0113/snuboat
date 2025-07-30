// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__BUILDER_HPP_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "snumsg_pkg/msg/detail/sensor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace snumsg_pkg
{

namespace msg
{

namespace builder
{

class Init_Sensor_vel
{
public:
  explicit Init_Sensor_vel(::snumsg_pkg::msg::Sensor & msg)
  : msg_(msg)
  {}
  ::snumsg_pkg::msg::Sensor vel(::snumsg_pkg::msg::Sensor::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::snumsg_pkg::msg::Sensor msg_;
};

class Init_Sensor_pose
{
public:
  explicit Init_Sensor_pose(::snumsg_pkg::msg::Sensor & msg)
  : msg_(msg)
  {}
  Init_Sensor_vel pose(::snumsg_pkg::msg::Sensor::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Sensor_vel(msg_);
  }

private:
  ::snumsg_pkg::msg::Sensor msg_;
};

class Init_Sensor_status
{
public:
  explicit Init_Sensor_status(::snumsg_pkg::msg::Sensor & msg)
  : msg_(msg)
  {}
  Init_Sensor_pose status(::snumsg_pkg::msg::Sensor::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Sensor_pose(msg_);
  }

private:
  ::snumsg_pkg::msg::Sensor msg_;
};

class Init_Sensor_tick
{
public:
  Init_Sensor_tick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sensor_status tick(::snumsg_pkg::msg::Sensor::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_Sensor_status(msg_);
  }

private:
  ::snumsg_pkg::msg::Sensor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::snumsg_pkg::msg::Sensor>()
{
  return snumsg_pkg::msg::builder::Init_Sensor_tick();
}

}  // namespace snumsg_pkg

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__BUILDER_HPP_
