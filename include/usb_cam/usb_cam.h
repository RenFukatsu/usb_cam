#ifndef __USB_CAM_H
#define __USB_CAM_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "yaml-cpp/yaml.h"

class UsbCam : public rclcpp::Node
{
public:
    UsbCam();
    void process();

private:
    void show_camera_parameter();
    template<typename T>
    void show_array(T array);
    sensor_msgs::msg::CameraInfo yaml_to_CameraInfo(std::string path);
    void set_camera_parameter();

    bool show_image;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    cv::VideoCapture camera;
    cv_bridge::CvImagePtr bridge;
};


#endif // __USB_CAM_H