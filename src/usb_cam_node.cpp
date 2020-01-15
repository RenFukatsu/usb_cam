#include "usb_cam/usb_cam.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<UsbCam> usb_cam = std::make_shared<UsbCam>();
    usb_cam->process();
    rclcpp::shutdown();
    return 0;
}