#include "usb_cam/image_listener.h"

ImageListener::ImageListener() : Node("image_listener")
{
    this->image_sub = this->create_subscription<sensor_msgs::msg::Image>("usb_cam/image_raw", 1, std::bind(&ImageListener::image_callback, this, std::placeholders::_1));
}

void ImageListener::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat cv_image = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::imshow("image_listener image", cv_image);
    cv::waitKey(5);
    return;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageListener>());
    rclcpp::shutdown();
    return 0;
}

