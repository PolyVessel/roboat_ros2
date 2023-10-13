cmake_minimum_required(VERSION 3.8)
project(roboat_interfaces)

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FutureMsg.msg"
  DEPENDENCIES std_msgs # Add packages that above messages depend on
)

ament_package()