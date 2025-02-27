cmake_minimum_required(VERSION 3.8)
project(answer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# 明确设置包含目录
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${info_interfaces_INCLUDE_DIRS}
)

# 添加库文件
add_library(${PROJECT_NAME}_lib
  src/game_state.cpp
  src/path_planner.cpp
  src/my_serial.cpp
  src/robot_controller.cpp
  src/image_processor.cpp
)

# 设置库的包含目录
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${info_interfaces_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)

# 链接库的依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  info_interfaces
  OpenCV
  cv_bridge
  sensor_msgs
)

# 添加可执行文件
add_executable(answer_node 
    src/main.cpp
        game_state.cpp
        image_processor.cpp
        my_serial.cpp
        path_planner.cpp
        robot_controller.cpp
)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}_lib
  answer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
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
