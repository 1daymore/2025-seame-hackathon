cmake_minimum_required(VERSION 3.22)        # ✅ 이 줄 추가
project(ros2_cv_webcam_cpp)                 # ✅ 이 줄 추가

# 기본 설정
set(CMAKE_CXX_STANDARD 17)

# 종속 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 실행 노드: img_publisher
add_executable(img_publisher src/image_publisher.cpp)
ament_target_dependencies(img_publisher rclcpp sensor_msgs cv_bridge)
target_link_libraries(img_publisher ${OpenCV_LIBS})

# 실행 노드: img_subscriber
add_executable(img_subscriber src/lane_detect_subscriber.cpp)
ament_target_dependencies(img_subscriber rclcpp sensor_msgs cv_bridge)
target_link_libraries(img_subscriber ${OpenCV_LIBS})

# 설치 설정
install(TARGETS
  img_publisher
  img_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()