#ifndef RM_CAMERA_DAHUA_DAHUA_CAMERA_HPP_
#define RM_CAMERA_DAHUA_DAHUA_CAMERA_HPP_



#include<iostream>
#include<stdlib.h>
#include<unistd.h>
#include<thread>
#include"rm_camera_dahua/Camera_Control.h"

#include "rm_utils/heartbeat.hpp"
#include<opencv2/opencv.hpp>

#include<std_msgs/msg/header.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include<vector>

namespace pka::camera_driver
{
    class Dahua_CameraNode : public rclcpp::Node
    {
    public:
        // 相机节点构造函数
        explicit Dahua_CameraNode(const rclcpp::NodeOptions &options);
        // 相机节点析构函数
        ~Dahua_CameraNode()override;
        // 搜索并连接相机
        bool Check_connect();
        // 取帧
        void CaptureImage();
        int empty_frame;
        // 打开相机（包括搜索和连接，拉流等）
        bool open();
        // 关闭相机（包括断开拉流，断开连接等）
        bool close();

    private:
        // Heartbeat
        // 心跳节点
        HeartBeatPublisher::SharedPtr heartbeat_;
        // Param set callback
        rcl_interfaces::msg::SetParametersResult onSetParameters(
            std::vector<rclcpp::Parameter> parameters);
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

        // Watch dog
        void timerCallback();
        rclcpp::TimerBase::SharedPtr timer_;
        int openCamera_error = 0;

        // Camera thread
        shared_ptr<std::thread> work_thread_;

        // General(借鉴FYT)
        bool is_open_ = false;

        /*==========相机参数===============*/
        // 相机图像格式
        std::string pixel_format_;
        uint64_t GSV_pixel_format_;

        double exposure_time_;   // 相机曝光时间
        double gain_;            // 相机增益
        int brightness;          // 相机亮度
        int auto_white_balance_; // 是否开启自动白平衡
        double frame_rate_;      // 帧率

        // ROI
        int64_t offset_x_;
        int64_t offset_y_;
        int64_t resolution_width_;
        int64_t resolution_height_;

        // Dahua camera
        Camera_Control camera;              // 定义Video类对象
        int connect_state = 0;     // 连接初始化状态
        Mat src;                   // 获取的帧图片
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        image_transport::CameraPublisher camera_pub_;
        sensor_msgs::msg::Image image_msg_;
        std::string camera_name_, camera_info_url_, frame_id_;
    };
} // namespace pka::camera_driver
#endif // endif RM_CAMERA_DAHUA_DAHUA_CAMERA_HPP_
