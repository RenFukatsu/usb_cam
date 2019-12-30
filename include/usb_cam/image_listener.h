#ifndef __LISTEN_IMAGE_H
#define __LISTEN_IMAGE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageListener : public rclcpp::Node
{
public:
    ImageListener();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};


#endif // __LISTEN_IMAGE_H