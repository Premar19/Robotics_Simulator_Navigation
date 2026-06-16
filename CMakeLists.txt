cmake_minimum_required(VERSION 3.8)
project(prs49_ros366_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros366_cv_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(helloWorld src/helloWorld.cpp)
add_executable(examplePublisher src/examplePublisher.cpp)
add_executable(lidarSub src/lidarSub.cpp)
add_executable(task src/task.cpp)
add_executable(ws3 src/ws3.cpp)
add_executable(ws4 src/ws4.cpp)
add_executable(colorClient src/colourClient.cpp)
add_executable(assignment src/assignment.cpp)
add_executable(ws5 src/ws5.cpp)

target_include_directories(helloWorld PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
  add_definitions(${OpenCV_DEFINITIONS})
target_compile_features(helloWorld PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(examplePublisher rclcpp std_msgs)
ament_target_dependencies(lidarSub rclcpp sensor_msgs)
ament_target_dependencies(task rclcpp sensor_msgs)
ament_target_dependencies(ws3 rclcpp sensor_msgs)
ament_target_dependencies(ws4 rclcpp sensor_msgs ros366_cv_msgs rcl_interfaces)
ament_target_dependencies(colorClient rclcpp sensor_msgs)
ament_target_dependencies(assignment rclcpp sensor_msgs ros366_cv_msgs rcl_interfaces image_transport cv_bridge OpenCV tf2 tf2_ros)
ament_target_dependencies(ws5 rclcpp sensor_msgs ros366_cv_msgs rcl_interfaces image_transport cv_bridge OpenCV tf2 tf2_ros)

install(TARGETS
  helloWorld
  examplePublisher
  lidarSub
  task
  ws3
  ws4
  colorClient
  assignment
  ws5
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
launch config
DESTINATION share/${PROJECT_NAME}/
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
