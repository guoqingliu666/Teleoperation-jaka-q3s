cmake_minimum_required(VERSION 3.8)
project(hand_control_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandJointState.msg"
  "srv/ControlMotors.srv"
  "srv/GetAngle.srv"
  "srv/GetAngularVelocity.srv"
  "srv/GetCanNodeId.srv"
  "srv/GetCanfdArbBaudrate.srv"
  "srv/GetCanfdDataBaudrate.srv"
  "srv/GetControlMode.srv"
  "srv/GetEnable.srv"
  "srv/GetFirmwareVersion.srv"
  "srv/GetHomeCurrent.srv"
  "srv/GetMaxCurrent.srv"
  "srv/GetNowAlarm.srv"
  "srv/GetNowAngle.srv"
  "srv/GetNowAngularVelocity.srv"
  "srv/GetNowCurrent.srv"
  "srv/GetNowPosition.srv"
  "srv/GetNowPositionVelocity.srv"
  "srv/GetNowStatus.srv"
  "srv/GetPosition.srv"
  "srv/GetPositionVelocity.srv"
  "srv/GetPositionReached.srv"
  "srv/GetRs485Baudrate.srv"
  "srv/GetRs485NodeId.srv"
  "srv/GetSafeCurrentEnable.srv"
  "srv/GetSerialNumber.srv"
  "srv/GetTorqueReached.srv"
  "srv/HomeMotors.srv"
  "srv/MoveMotors.srv"
  "srv/PlayGesture.srv"
  "srv/SetAngle.srv"
  "srv/SetAngularVelocity.srv"
  "srv/SetCanNodeId.srv"
  "srv/SetCanfdArbBaudrate.srv"
  "srv/SetCanfdDataBaudrate.srv"
  "srv/SetClearAlarm.srv"
  "srv/SetControlMode.srv"
  "srv/SetEnable.srv"
  "srv/SetHomeCurrent.srv"
  "srv/SetMaxCurrent.srv"
  "srv/SetMoveNoHome.srv"
  "srv/SetPosition.srv"
  "srv/SetPositionVelocity.srv"
  "srv/SetRs485Baudrate.srv"
  "srv/SetRs485NodeId.srv"
  "srv/SetSafeCurrentEnable.srv"
  "srv/StopMotors.srv"
  DEPENDENCIES std_msgs
)

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

ament_package()
