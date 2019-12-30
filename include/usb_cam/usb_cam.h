#ifndef __USB_CAM_H
#define __USB_CAM_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class UsbCam : public rclcpp::Node
{
public:
    UsbCam();

private:
    void set_camera_parameter();
    void process();

    bool show_image;
    int width;
    int height;

    cv::VideoCapture camera;
    int count = 0;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
};


#endif // __USB_CAM_H