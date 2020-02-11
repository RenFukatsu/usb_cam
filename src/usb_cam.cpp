#include "usb_cam/usb_cam.h"

UsbCam::UsbCam() : Node("usb_cam"), bridge(std::make_shared<cv_bridge::CvImage>())
{
    this->bridge->encoding = sensor_msgs::image_encodings::BGR8;
    this->bridge->header.frame_id = "usb_cam";

    this->show_image = this->declare_parameter("show_image", false);
    std::string yaml_path = this->declare_parameter("yaml_path", "camera.yaml");

    this->camera_info_msg = this->yaml_to_CameraInfo(yaml_path);
    this->camera_info_msg.header.frame_id = "usb_cam";

    std::cout << "------ usb_cam ------" << std::endl;
    std::cout << "show image : " << (this->show_image ? "true" : "false")  << std::endl;
    std::cout << "yaml_path : " << yaml_path << std::endl;
    this->show_camera_parameter();

    this->camera.open(cv::CAP_ANY);
    if(!camera.isOpened())
    {
        RCUTILS_LOG_FATAL("camera is not open");
        rclcpp::shutdown();
        return;
    }
    this->set_camera_parameter();
}

template<typename T>
void UsbCam::show_array(T array)
{
    for(size_t i=0; i<array.size(); i++)
    {
        std::cout << array[i];
        if(i != array.size() - 1)
        {
            std::cout << ", ";
        }
    }
}

sensor_msgs::msg::CameraInfo UsbCam::yaml_to_CameraInfo(std::string path)
{
    sensor_msgs::msg::CameraInfo msg;
    try
    {
        YAML::Node param = YAML::LoadFile(path);
        msg.width = param["image_width"].as<uint32_t>();
        msg.height = param["image_height"].as<uint32_t>();
        msg.header.frame_id = param["camera_name"].as<std::string>();
        std::vector<double> k = param["camera_matrix"]["data"].as<std::vector<double>>();
        std::copy_n(k.begin(), 9UL, msg.k.begin());
        msg.d = param["distortion_coefficients"]["data"].as<std::vector<double>>();
        std::vector<double> r = param["rectification_matrix"]["data"].as<std::vector<double>>();
        std::copy_n(r.begin(), 9UL, msg.r.begin());
        std::vector<double> p = param["projection_matrix"]["data"].as<std::vector<double>>();
        std::copy_n(p.begin(), 12UL, msg.p.begin());
        msg.distortion_model = param["distortion_model"].as<std::string>();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return msg;
}

void UsbCam::show_camera_parameter()
{
    std::cout << "width : " << this->camera_info_msg.width << std::endl;
    std::cout << "height : " << this->camera_info_msg.height << std::endl;
    std::cout << "k : [";
    this->show_array(this->camera_info_msg.k);
    std::cout << "]" << std::endl;
    std::cout << "d : [";
    this->show_array(this->camera_info_msg.d);
    std::cout << "]" << std::endl;
    std::cout << "r : [";
    this->show_array(this->camera_info_msg.r);
    std::cout << "]" << std::endl;
    std::cout << "p : [";
    this->show_array(this->camera_info_msg.p);
    std::cout << "]" << std::endl;
    std::cout << "distortion_model : " << this->camera_info_msg.distortion_model << std::endl;
}

void UsbCam::set_camera_parameter()
{
    try
    {
        this->camera.set(CV_CAP_PROP_FRAME_WIDTH, this->camera_info_msg.width);
        this->camera.set(CV_CAP_PROP_FRAME_HEIGHT, this->camera_info_msg.height);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


void UsbCam::process()
{
    image_transport::ImageTransport it(shared_from_this());
    image_transport::CameraPublisher image_pub = it.advertiseCamera("usb_cam/image_raw", 1);
    while (rclcpp::ok())
    {
        cv::Mat cv_image;
        this->camera >> cv_image;
        if(this->show_image)
        {
            cv::imshow("usb_cam image", cv_image);
            cv::waitKey(5);
        }

        rclcpp::Time timestamp = this->get_clock()->now();
        this->camera >> this->bridge->image;
        if(this->bridge->image.empty())
        {
            RCUTILS_LOG_ERROR("no frame");
            continue;
        }
        this->bridge->header.stamp = timestamp;
        this->camera_info_msg.header.stamp = timestamp;
        image_pub.publish(this->bridge->toImageMsg(), sensor_msgs::msg::CameraInfo::ConstPtr(new sensor_msgs::msg::CameraInfo(this->camera_info_msg)));
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<UsbCam> usb_cam = std::make_shared<UsbCam>();
    usb_cam->process();
    rclcpp::shutdown();
    return 0;
}

