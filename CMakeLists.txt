cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

add_executable(usb_cam src/usb_cam.cpp)
ament_target_dependencies(usb_cam
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
    yaml-cpp
    image_transport
)
target_link_libraries(usb_cam
    yaml-cpp
)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher
    rclcpp
    sensor_msgs
    yaml-cpp
)
target_link_libraries(camera_info_publisher
    yaml-cpp
)

add_executable(image_listener src/image_listener.cpp)
ament_target_dependencies(image_listener
    rclcpp
    std_msgs
    sensor_msgs
    cv_bridge
)

install(TARGETS
    usb_cam
    camera_info_publisher
    image_listener
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    parameters
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
