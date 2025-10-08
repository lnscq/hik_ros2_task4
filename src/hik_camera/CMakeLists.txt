cmake_minimum_required(VERSION 3.8)
project(hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS /opt/MVS/include
  NO_DEFAULT_PATH
)

find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS /opt/MVS/lib/64
  NO_DEFAULT_PATH
)

if(MVS_INCLUDE_DIR AND MVS_LIBRARY)
  message(STATUS "Found Hikvision MVS SDK: ${MVS_INCLUDE_DIR}, ${MVS_LIBRARY}")
else()
  message(FATAL_ERROR "Hikvision MVS SDK not found!")
endif()

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

# 添加可执行文件
add_executable(hik_camera_node
  src/hik_camera_node.cpp
  src/main.cpp
)

target_include_directories(hik_camera_node PRIVATE
  include
  ${MVS_INCLUDE_DIR}
)

target_link_libraries(hik_camera_node
  cv_bridge::cv_bridge
  image_transport::image_transport
  camera_info_manager::camera_info_manager
  rclcpp::rclcpp
  ${sensor_msgs_LIBRARIES}
  ${MVS_LIBRARY}
  ${OpenCV_LIBRARIES}
)

# 安装目标
install(TARGETS
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
