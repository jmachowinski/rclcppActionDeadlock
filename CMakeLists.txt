cmake_minimum_required(VERSION 3.8)
project(deadlock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    action/SomeAction.action
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(server src/Server.cpp)
ament_target_dependencies(server PUBLIC rclcpp rclcpp_action)
target_link_libraries(server PRIVATE ${cpp_typesupport_target})

add_executable(client src/Client.cpp)
ament_target_dependencies(client PUBLIC rclcpp rclcpp_action)
target_link_libraries(client PRIVATE ${cpp_typesupport_target})


ament_package()
