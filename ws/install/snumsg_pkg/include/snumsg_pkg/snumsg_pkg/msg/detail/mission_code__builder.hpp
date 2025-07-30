// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__BUILDER_HPP_
#define SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "snumsg_pkg/msg/detail/mission_code__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace snumsg_pkg
{

namespace msg
{

namespace builder
{

class Init_MissionCode_mission_code
{
public:
  explicit Init_MissionCode_mission_code(::snumsg_pkg::msg::MissionCode & msg)
  : msg_(msg)
  {}
  ::snumsg_pkg::msg::MissionCode mission_code(::snumsg_pkg::msg::MissionCode::_mission_code_type arg)
  {
    msg_.mission_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::snumsg_pkg::msg::MissionCode msg_;
};

class Init_MissionCode_tick
{
public:
  Init_MissionCode_tick()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissionCode_mission_code tick(::snumsg_pkg::msg::MissionCode::_tick_type arg)
  {
    msg_.tick = std::move(arg);
    return Init_MissionCode_mission_code(msg_);
  }

private:
  ::snumsg_pkg::msg::MissionCode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::snumsg_pkg::msg::MissionCode>()
{
  return snumsg_pkg::msg::builder::Init_MissionCode_tick();
}

}  // namespace snumsg_pkg

#endif  // SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__BUILDER_HPP_
