#include "usb_cam/camera_info_publisher.h"

CameraInfoPublisher::CameraInfoPublisher() : Node("camera_info_publisher")
{
    this->camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);

    std::string yaml_path = this->declare_parameter("yaml_path", "camera.yaml");
    this->camera_info_msg.header.frame_id = "usb_cam";
    this->camera_info_msg = this->yaml_to_CameraInfo(yaml_path);

    this->timer = this->create_wall_timer(500ms, std::bind(&CameraInfoPublisher::timer_callback, this));

    std::cout << "------ camera_info_publisher ------" << std::endl;
    std::cout << "yaml_path : " << yaml_path << std::endl;
    this->show_camera_parameter();
}

void CameraInfoPublisher::timer_callback()
{
    rclcpp::Time timestamp = this->get_clock()->now();
    camera_info_msg.header.stamp = timestamp;
    this->camera_info_pub->publish(this->camera_info_msg);
}

void CameraInfoPublisher::show_camera_parameter()
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

template<typename T>
void CameraInfoPublisher::show_array(T array)
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

sensor_msgs::msg::CameraInfo CameraInfoPublisher::yaml_to_CameraInfo(std::string path)
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInfoPublisher>());
    rclcpp::shutdown();
    return 0;
}

