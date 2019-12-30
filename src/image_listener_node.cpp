#include "usb_cam/image_listener.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageListener>());
    rclcpp::shutdown();
    return 0;
}