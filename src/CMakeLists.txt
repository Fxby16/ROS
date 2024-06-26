cmake_minimum_required(VERSION 3.8)
project(ROS_PickingPoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Iconv REQUIRED)
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} pickingPoint/include/)
file(GLOB SOURCES "pickingPoint/src/*.cpp")

add_executable(applicant src/applicant.cpp)
ament_target_dependencies(applicant rclcpp std_msgs image_transport cv_bridge)
# Link OpenCV libraries to the applicant executable
target_link_libraries(applicant ${OpenCV_LIBRARIES})

add_executable(pickingPointHandler src/pickingPointHandler.cpp ${SOURCES})
target_link_libraries(pickingPointHandler ${OpenCV_LIBRARIES} ${VTK_LIBRARIES} ${Eigen3_LIBRARIES} ${Iconv_LIBRARIES})
ament_target_dependencies(pickingPointHandler rclcpp std_msgs image_transport cv_bridge)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

install(TARGETS
  applicant
  pickingPointHandler
  receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()