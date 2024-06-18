cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package( OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} pickingPoint/include/ )
file(GLOB SOURCES "pickingPoint/src/*.cpp")

add_executable(applicant src/applicant.cpp)
ament_target_dependencies(applicant rclcpp std_msgs image_transport cv_bridge)

add_executable(pickingPointHandler src/pickingPointHandler.cpp ${SOURCES})
target_link_libraries(pickingPointHandler ${OpenCV_LIBS})
ament_target_dependencies(pickingPointHandler rclcpp std_msgs image_transport cv_bridge)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

install(TARGETS
  applicant
  pickingPointHandler
  receiver
  DESTINATION lib/${PROJECT_NAME})


ament_package()
