#include "usb_cam/usb_cam.h"

UsbCam::UsbCam() : Node("usb_cam")
{
    this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("usb_cam/image_raw", 1);

    this->show_image = this->declare_parameter("show_image", false);

    std::cout << "------ usb_cam ------" << std::endl;
    std::cout << "show image : " << (this->show_image ? "true" : "false")  << std::endl;

    this->camera.open(cv::CAP_ANY);
    if(!camera.isOpened())
    {
        RCUTILS_LOG_FATAL("camera is not open");
        rclcpp::shutdown();
        return;
    }
    this->process();
}

void UsbCam::process()
{
    while (rclcpp::ok())
    {
        cv::Mat cv_image;
        rclcpp::Time timestamp = this->get_clock()->now();
        this->camera >> cv_image;
        if(cv_image.empty())
        {
            RCUTILS_LOG_ERROR("no frame");
            continue;
        }
        std_msgs::msg::Header header;
        header.frame_id = this->count++;
        header.stamp = timestamp;
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", cv_image).toImageMsg();
        this->image_pub->publish(*msg);

        if(this->show_image)
        {
            cv::imshow("usb_cam image", cv_image);
            cv::waitKey(5);
        }
    }
}