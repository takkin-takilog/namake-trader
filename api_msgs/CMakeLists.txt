cmake_minimum_required(VERSION 3.5)
project(api_msgs)

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
  "msg/Candle.msg"
  "msg/FailReasonCode.msg"
  "msg/Granularity.msg"
  "msg/Instrument.msg"
  "msg/OrderState.msg"
  "msg/OrderType.msg"
  "msg/PriceBucket.msg"
  "msg/Pricing.msg"
  "msg/ProfitLossOrder.msg"
  "msg/TradeState.msg"
  "srv/CandlesSrv.srv"
  "srv/OrderCancelSrv.srv"
  "srv/OrderCreateSrv.srv"
  "srv/OrderDetailsSrv.srv"
  "srv/TradeCloseSrv.srv"
  "srv/TradeCRCDOSrv.srv"
  "srv/TradeDetailsSrv.srv"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
