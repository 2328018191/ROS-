cmake_minimum_required(VERSION 3.8)
project(cpp_service)

# 基础编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------
# 1. 依赖查找
# ------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# 来自 cpp_tf 的依赖
find_package(rclcpp            REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs     REQUIRED)

# 来自 cpp_service 的额外依赖
find_package(chapt   REQUIRED)
find_package(turtlesim REQUIRED)

# ------------------------------------------------------------
# 2. 生成可执行文件
# ------------------------------------------------------------
# tf 相关节点
add_executable(tf_static src/tf_static.cpp)
ament_target_dependencies(tf_static
  rclcpp tf2_ros tf2_geometry_msgs geometry_msgs
)

# service/client 节点
add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control
  chapt rclcpp geometry_msgs turtlesim
)

add_executable(turtle_client  src/turtle_client.cpp)
ament_target_dependencies(turtle_client
  chapt rclcpp
)

# ------------------------------------------------------------
# 3. 安装
# ------------------------------------------------------------
# 安装可执行文件
install(TARGETS tf_static turtle_control turtle_client
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------
# 4. 测试（按需保留）
# ------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------
# 5. 导出
# ------------------------------------------------------------
ament_package()