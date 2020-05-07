cmake_minimum_required(VERSION 3.5)
project(trade_manager_msgs)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarketOrderRequest.msg"
  "msg/LimitOrderRequest.msg"
  "msg/StopOrderRequest.msg"
  "msg/CandleMnt.msg"
  "msg/GranularityMnt.msg"
  "msg/InstrumentMnt.msg"
  "srv/CandlesMntSrv.srv"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
