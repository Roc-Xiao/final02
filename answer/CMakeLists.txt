cmake_minimum_required(VERSION 3.8)
project(answer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
message(STATUS "CMAKE_PREFIX_PATH: $ENV{CMAKE_PREFIX_PATH}")
message(STATUS "Looking for info_interfaces...")
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(info_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(example_interfaces REQUIRED)

# 明确设置包含目录
include_directories(
  include
)

# 添加可执行文件
add_executable(game_node
    src/main.cpp
    src/game_state.cpp
    src/image_processor.cpp
    src/path_planner.cpp
    src/robot_controller.cpp
)
# 链接库的依赖
ament_target_dependencies(game_node
  rclcpp
  std_msgs
  geometry_msgs
  info_interfaces
  OpenCV
  cv_bridge
  sensor_msgs
  example_interfaces
)

# 安装目标
install(TARGETS
  game_node
  RUNTIME DESTINATION libexec/${PROJECT_NAME}
)

# 添加对launch文件的支持
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package(
)
