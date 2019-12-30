#ifndef __CAMERA_INFO_PUBLISHER_H
#define __CAMERA_INFO_PUBLISHER_H

#include <chrono>
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class CameraInfoPublisher : public rclcpp::Node
{
public:
    CameraInfoPublisher();

private:
    void show_camera_parameter();
    template<typename T>
    void show_array(T array);
    void timer_callback();
    sensor_msgs::msg::CameraInfo yaml_to_CameraInfo(std::string path);

    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
    rclcpp::TimerBase::SharedPtr timer;
};


#endif // __LISTEN_IMAGE_H