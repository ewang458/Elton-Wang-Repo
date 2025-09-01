cmake_minimum_required(VERSION 3.8)
project(dht11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(temp_sub src/temp_sub.cpp)
add_executable(humid_sub src/humid_sub.cpp)
add_executable(dht_pub src/dht_pub.cpp)
ament_target_dependencies(temp_sub rclcpp std_msgs)
ament_target_dependencies(humid_sub rclcpp std_msgs)
ament_target_dependencies(dht_pub rclcpp std_msgs)

install(TARGETS
  temp_sub
  humid_sub
  dht_pub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
