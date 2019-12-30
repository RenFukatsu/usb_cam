#include "usb_cam/usb_cam.h"

UsbCam::UsbCam() : Node("usb_cam")
{
    this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("usb_cam/image_raw", 1);

    this->show_image = this->declare_parameter("show_image", false);
    this->width = this->declare_parameter("width", 640);
    this->height = this->declare_parameter("height", 480);

    std::cout << "------ usb_cam ------" << std::endl;
    std::cout << "show image : " << (this->show_image ? "true" : "false")  << std::endl;
    std::cout << "width : " << this->width << std::endl;
    std::cout << "height : " << this->height << std::endl;

    this->camera.open(cv::CAP_ANY);
    if(!camera.isOpened())
    {
        RCUTILS_LOG_FATAL("camera is not open");
        rclcpp::shutdown();
        return;
    }
    this->set_camera_parameter();

    this->process();
}

void UsbCam::set_camera_parameter()
{
    try
    {
        this->camera.set(CV_CAP_PROP_FRAME_WIDTH, this->width);
        this->camera.set(CV_CAP_PROP_FRAME_HEIGHT, this->height);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
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