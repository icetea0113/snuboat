cmake_minimum_required(VERSION 3.8)
project(snumsg_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 추가: Time 메시지 사용을 위해 builtin_interfaces 찾기
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)            # ← 이 줄 추가
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionCode.msg"
  "msg/Sensor.msg"
  DEPENDENCIES
    geometry_msgs
    builtin_interfaces                              # ← 이 줄 추가
)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
