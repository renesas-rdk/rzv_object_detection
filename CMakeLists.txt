cmake_minimum_required(VERSION 3.8)
project(rzv_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rzv_model REQUIRED)
find_package(rzv_yolox REQUIRED)
find_package(rzv_yolov8 REQUIRED)
find_package(rzv_gold_yolo REQUIRED)
find_package(rzv_model_utils_ros2 REQUIRED)

# Shared include directories
set(SHARED_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# create target
set(TARGET_OBJ_DET_YOLOX yolox_object_detection)
add_executable(${TARGET_OBJ_DET_YOLOX}
  src/yolox_object_detection_node.cpp
)

# include directories
target_include_directories(${TARGET_OBJ_DET_YOLOX} PUBLIC ${SHARED_INCLUDE_DIRS})

# ROS2 dependencies
ament_target_dependencies(${TARGET_OBJ_DET_YOLOX}
  rclcpp
  sensor_msgs
  geometry_msgs
  rzv_model
  rzv_yolox
  rzv_model_utils_ros2
)

# create target
set(TARGET_OBJ_DET_YOLOv8 yolov8_object_detection)
add_executable(${TARGET_OBJ_DET_YOLOv8}
  src/yolov8_object_detection_node.cpp
)

# include directories
target_include_directories(${TARGET_OBJ_DET_YOLOv8} PUBLIC ${SHARED_INCLUDE_DIRS})

# ROS2 dependencies
ament_target_dependencies(${TARGET_OBJ_DET_YOLOv8}
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  rzv_model
  rzv_yolov8
  rzv_model_utils_ros2
)

# create target
set(TARGET_OBJ_DET_GOLD_YOLO gold_yolo_object_detection)
add_executable(${TARGET_OBJ_DET_GOLD_YOLO}
  src/gold_yolo_object_detection_node.cpp
)

# include directories
target_include_directories(${TARGET_OBJ_DET_GOLD_YOLO} PUBLIC ${SHARED_INCLUDE_DIRS})

# ROS2 dependencies
ament_target_dependencies(${TARGET_OBJ_DET_GOLD_YOLO}
  rclcpp
  sensor_msgs
  geometry_msgs
  rzv_model
  rzv_gold_yolo
  rzv_model_utils_ros2
)

install(
  TARGETS ${TARGET_OBJ_DET_YOLOX} ${TARGET_OBJ_DET_YOLOv8} ${TARGET_OBJ_DET_GOLD_YOLO}
  EXPORT rzv_object_detection_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(rzv_object_detection_targets)
ament_export_targets(rzv_object_detection_targets)

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
