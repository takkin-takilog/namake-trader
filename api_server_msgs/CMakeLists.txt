cmake_minimum_required(VERSION 3.8)
project(api_server_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
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
)

set(srv_files
  "srv/AccountQuerySrv.srv"
  "srv/CandlesQuerySrv.srv"
  "srv/OrderCancelSrv.srv"
  "srv/OrderCreateSrv.srv"
  "srv/OrderDetailsSrv.srv"
  "srv/PricingQuerySrv.srv"
  "srv/TradeCloseSrv.srv"
  "srv/TradeCRCDOSrv.srv"
  "srv/TradeDetailsSrv.srv"
)

set(act_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${act_files}
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
