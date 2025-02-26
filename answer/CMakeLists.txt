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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(info_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../info_interfaces/include
)

add_executable(answer_node
        src/main.cpp
        src/my_serial.cpp
        src/game_state.cpp
        src/path_planner.cpp
        src/robot_controller.cpp
        src/image_processor.cpp
)

ament_target_dependencies(answer_node
        rclcpp
        sensor_msgs
        std_msgs
        geometry_msgs
        info_interfaces
        OpenCV
        cv_bridge
)

target_link_libraries(answer_node
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  info_interfaces::info_interfaces
  ${OpenCV_LIBS}
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

install(TARGETS
        answer_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
