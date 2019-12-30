#include "usb_cam/camera_info_publisher.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}