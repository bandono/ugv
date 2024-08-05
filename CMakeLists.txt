cmake_minimum_required(VERSION 3.8)
project(ugv)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/Command.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

include_directories(include)  

add_executable(vehicle src/ugv.cpp)
ament_target_dependencies(vehicle rclcpp std_msgs)

add_executable(ground src/ground.cpp)
ament_target_dependencies(ground rclcpp std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(vehicle "${cpp_typesupport_target}")
target_link_libraries(ground "${cpp_typesupport_target}")

install(TARGETS
  vehicle
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ground
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
