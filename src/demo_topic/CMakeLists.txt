cmake_minimum_required(VERSION 3.8)
project(demo_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CURL REQUIRED)

add_executable(novel_pub
  novel_pub.cpp
)

ament_target_dependencies(novel_pub rclcpp std_msgs)
target_link_libraries(novel_pub CURL::libcurl)

install(TARGETS
  novel_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()