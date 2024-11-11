#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
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
        _config = LoadConfig("/home/wl/Documents/detector/src/camera_node/config/config.yaml");
        
        _camera_params = LoadCameraParameters(_config._camera_config_path);

        // 设置发布器
        image_publisher_ = image_transport::create_publisher(this, "image");

        // 打开USB摄像头
        cap_.open(_config._device_port, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, _config._frame_width);  //设置捕获视频的宽度
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, _config._frame_height);  //设置捕获视频的高度
        cap_.set(cv::CAP_PROP_FPS, _config._camera_fps);
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
        int _frame_width;
        int _frame_height;
        int _camera_fps;
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
    
    Config LoadConfig(std::string config_path)
    {
        YAML::Node yaml = YAML::LoadFile(config_path);
        Config config;
        config._undistort = yaml["undistort"].as<bool>();
        config._camera_config_path = yaml["camera_config_path"].as<std::string>(), 
        config._device_port = yaml["device_port"].as<std::string>(), 
        config._pub_time_interval = yaml["pub_time_interval"].as<int>(), 
        config._frame_width = yaml["frame_width"].as<int>(),
        config._frame_height = yaml["frame_height"].as<int>(),
        config._camera_fps = yaml["camera_fps"].as<int>();
        return config;
    };
    
    CameraParams LoadCameraParameters(std::string camera_params_path)
    {
        // 从YAML文件加载相机参数
        YAML::Node yaml = YAML::LoadFile(camera_params_path);
        CameraParams cam_params;
        cam_params._camera_matrix.at<double>(0,0) = yaml["fx"].as<double>();
        cam_params._camera_matrix.at<double>(1,1) = yaml["fy"].as<double>();
        cam_params._camera_matrix.at<double>(0,2) = yaml["cx"].as<double>();
        cam_params._camera_matrix.at<double>(1,2) = yaml["cy"].as<double>();
        cam_params._dist_coeffs.at<double>(0,0) = yaml["k1"].as<double>();
        cam_params._dist_coeffs.at<double>(0,1) = yaml["k2"].as<double>();
        cam_params._dist_coeffs.at<double>(0,2) = yaml["p1"].as<double>();
        cam_params._dist_coeffs.at<double>(0,3) = yaml["p2"].as<double>();
        cam_params._dist_coeffs.at<double>(0,4) = yaml["k3"].as<double>();
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
        
        if (_config._undistort)
        {
            std::cout << "image_undistorted" << std::endl;
            cv::undistort(frame, frame, _camera_params._camera_matrix, _camera_params._dist_coeffs);
        }

        // 将OpenCV图像转换为ROS消息
        std_msgs::msg::Header header;
        header.frame_id = "camera";
        header.stamp = this->now();
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        // 发布图像消息
        image_publisher_.publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;
    // 全局参数
    Config _config;
    // 相机内参
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
