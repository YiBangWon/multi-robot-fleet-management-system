cmake_minimum_required(VERSION 3.8)
project(fms_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tasks.msg"
  "msg/TaskState.msg"
  "msg/Robots.msg"
  "msg/RobotState.msg"
  "msg/TaskInfo.msg"
  "msg/Node.msg"
  "msg/RobotInfo.msg"
  "msg/RobotModel.msg"
  "msg/route_table_msg/NodeRT.msg"
  "msg/route_table_msg/RouteTableNode.msg"
  "msg/route_table_msg/AgentRouteTable.msg"
  "msg/route_table_msg/RouteTable.msg"
  "srv/CancelTask.srv"
  "srv/SubmitTask.srv"
  "srv/CommandRobot.srv"
  "srv/Terminate.srv"
  "srv/ActiveRobot.srv"
  "srv/DisableRegion.srv"

  "msg/RouteInfo.msg"
  "msg/CurrStep.msg"
  DEPENDENCIES std_msgs geometry_msgs
)



ament_package()
