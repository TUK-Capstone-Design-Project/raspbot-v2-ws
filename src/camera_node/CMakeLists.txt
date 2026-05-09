cmake_minimum_required(VERSION 4.0)
project(camera_node)

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)      # ROS-OpenCV 브릿지
find_package(OpenCV REQUIRED)         # OpenCV 라이브러리
find_package(tf2_geometry_msgs REQUIRED)  # [추가]

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(camera_node)

target_sources(camera_node PRIVATE
  src/main.cpp
)

ament_target_dependencies(camera_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  cv_bridge
  tf2_geometry_msgs
)

# 2. GStreamer 헤더 파일 경로 추가
target_include_directories(camera_node PRIVATE
  ${GST_INCLUDE_DIRS}
)

# 3. GStreamer 라이브러리 링크 추가
target_link_libraries(camera_node
  ${GST_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 4. GStreamer 컴파일/링크 디렉토리 플래그 추가 (옵션이지만 권장)
target_compile_options(camera_node PRIVATE ${GST_CFLAGS})
target_link_directories(camera_node PRIVATE ${GST_LIBRARY_DIRS})

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

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
