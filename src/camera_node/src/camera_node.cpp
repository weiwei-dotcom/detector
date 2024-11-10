#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // 读取相机参数

        Config _config = LoadConfig("/home/ww/Documents/detector/src/camera_node/config/config.yaml");
        
        CameraParams _camera_params = LoadCameraParameters(_config._camera_config_path);

        // 设置发布器
        image_publisher_ = image_transport::create_publisher(this, "image");

        // 打开USB摄像头
        cap_.open(_config._device_port);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera!");
            rclcpp::shutdown();
        }

        // 定时器，用于定时抓取和发布图像
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(_config._pub_time_interval), // 每30ms获取一帧图像
            std::bind(&CameraNode::captureAndPublish, this));
    }

private:
    struct Config
    {
        bool _undistort;
        std::string _camera_config_path;
        std::string _device_port;
        int _pub_time_interval;
        Config(const bool &undistort = false, 
            const std::string &camera_config_path = "/home/ww/Documents/detector/src/camera_node/config/camera_config.yaml", 
            const std::string &device_port = "/dev/vedio0",
            const int &pub_time_interval = 30) 
            : _undistort(false), _camera_config_path(camera_config_path), _device_port(device_port), _pub_time_interval(pub_time_interval){}
    };

    struct CameraParams
    {
        cv::Mat _camera_matrix = cv::Mat_<double>(3, 3);
        cv::Mat _dist_coeffs = cv::Mat_<double>(1, 5);
        CameraParams(
            const cv::Mat &camera_matrix = (cv::Mat_<double>(3, 3) << 1.0, 0, 320, 0, 1.0, 240, 0, 0, 1),
            const cv::Mat &dist_coeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0)) 
            :  _camera_matrix(camera_matrix), _dist_coeffs(dist_coeffs) {}
    };

    Config LoadConfig(const std::string &config_path)
    {
        std::cout << "here:63" << std::endl;
        YAML::Node yaml = YAML::LoadFile(config_path);
        std::cout << "here:66" << std::endl;

        Config config(yaml["undistort"].as<bool>(), yaml["camera_config_path"].as<std::string>(), yaml["device_port"].as<std::string>(), yaml["pub_time_interval"].as<int>());
        return config;
    };
    
    CameraParams LoadCameraParameters(const std::string &camera_params_path)
    {
        // 从YAML文件加载相机参数
        YAML::Node yaml = YAML::LoadFile(camera_params_path);
        CameraParams cam_params;
        cam_params._camera_matrix.at<double>(0,0) = yaml["fx"].as<double>();
        cam_params._camera_matrix.at<double>(1,1) = yaml["fy"].as<double>();
        cam_params._camera_matrix.at<double>(0,2) = yaml["cx"].as<double>();
        cam_params._camera_matrix.at<double>(1,2) = yaml["cy"].as<double>();
        cam_params._dist_coeffs.at<double>(0) = yaml["k1"].as<double>();
        cam_params._dist_coeffs.at<double>(1) = yaml["k2"].as<double>();
        cam_params._dist_coeffs.at<double>(2) = yaml["p1"].as<double>();
        cam_params._dist_coeffs.at<double>(3) = yaml["p2"].as<double>();
        cam_params._dist_coeffs.at<double>(4) = yaml["k3"].as<double>();
        return cam_params;
    }

    void captureAndPublish()
    {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Captured an empty frame!");
            return;
        }

        // 矫正图像
        if (_config._undistort)
            cv::undistort(frame, frame, _camera_params._camera_matrix, _camera_params._dist_coeffs);

        // 将OpenCV图像转换为ROS消息
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        // 发布图像消息
        image_publisher_.publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;

    // 全局参数
    Config _config;
    CameraParams _camera_params;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
