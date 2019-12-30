#include "usb_cam/usb_cam.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UsbCam>());
    rclcpp::shutdown();
    return 0;
}