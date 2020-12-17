#pragma once

#include <libuvc/libuvc.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <astra_camera/srv/get_uvc_exposure.hpp>
#include <astra_camera/srv/set_uvc_exposure.hpp>
#include <astra_camera/srv/get_uvc_gain.hpp>
#include <astra_camera/srv/set_uvc_gain.hpp>
#include <astra_camera/srv/get_uvc_white_balance.hpp>
#include <astra_camera/srv/set_uvc_white_balance.hpp>
#include <astra_camera/srv/get_camera_info.hpp>
#include <astra_camera/srv/get_device_type.hpp>
#include <astra_camera/astra_device_type.h>
#include <string>

namespace libuvc_camera {

class CameraDriver : public rclcpp::Node
{
public:
    CameraDriver();
    ~CameraDriver();

    bool Start();
    void Stop();
private:
    enum State {
        kInitial = 0,
        kStopped = 1,
        kRunning = 2,
    };

    void OpenCamera();
    void CloseCamera();

    void ReconfigureCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    enum uvc_frame_format GetVideoMode(std::string vmode);

    void AutoControlsCallback(enum uvc_status_class status_class,
                              int event,
                              int selector,
                              enum uvc_status_attribute status_attribute,
                              void *data, size_t data_len);
    static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                     int event,
                                     int selector,
                                     enum uvc_status_attribute status_attribute,
                                     void *data, size_t data_len,
                                     void *ptr);
    void ImageCallback(uvc_frame_t *frame);
    static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
    bool getUVCExposureCb(const std::shared_ptr<astra_camera::srv::GetUVCExposure::Request> request, std::shared_ptr<astra_camera::srv::GetUVCExposure::Response> response);
    bool setUVCExposureCb(const std::shared_ptr<astra_camera::srv::SetUVCExposure::Request> request, std::shared_ptr<astra_camera::srv::SetUVCExposure::Response> response);
    bool getUVCGainCb(const std::shared_ptr<astra_camera::srv::GetUVCGain::Request> request, std::shared_ptr<astra_camera::srv::GetUVCGain::Response> response);
    bool setUVCGainCb(const std::shared_ptr<astra_camera::srv::SetUVCGain::Request> request, std::shared_ptr<astra_camera::srv::SetUVCGain::Response> response);
    bool getUVCWhiteBalanceCb(const std::shared_ptr<astra_camera::srv::GetUVCWhiteBalance::Request> request, std::shared_ptr<astra_camera::srv::GetUVCWhiteBalance::Response> response);
    bool setUVCWhiteBalanceCb(const std::shared_ptr<astra_camera::srv::SetUVCWhiteBalance::Request> request, std::shared_ptr<astra_camera::srv::SetUVCWhiteBalance::Response> response);

    State state_;
    boost::recursive_mutex mutex_;

    uvc_context_t *ctx_;
    uvc_device_t *dev_;
    uvc_device_handle_t *devh_;
    uvc_frame_t *rgb_frame_;

    std::unique_ptr<image_transport::ImageTransport> it_ = nullptr;
    std::unique_ptr<image_transport::CameraPublisher> cam_pub_ = nullptr;

    rclcpp::AsyncParametersClient::SharedPtr param_client_;

    camera_info_manager::CameraInfoManager cinfo_manager_;
    bool param_init_;
    std::string ns;
    std::string ns_no_slash;

    rclcpp::Service<astra_camera::srv::GetUVCExposure>::SharedPtr get_uvc_exposure_server;
    rclcpp::Service<astra_camera::srv::SetUVCExposure>::SharedPtr set_uvc_exposure_server;
    rclcpp::Service<astra_camera::srv::GetUVCGain>::SharedPtr get_uvc_gain_server;
    rclcpp::Service<astra_camera::srv::SetUVCGain>::SharedPtr set_uvc_gain_server;
    rclcpp::Service<astra_camera::srv::GetUVCWhiteBalance>::SharedPtr get_uvc_white_balance_server;
    rclcpp::Service<astra_camera::srv::SetUVCWhiteBalance>::SharedPtr set_uvc_white_balance_server;

    rclcpp::Client<astra_camera::srv::GetDeviceType>::SharedPtr device_type_client;
    rclcpp::Client<astra_camera::srv::GetCameraInfo>::SharedPtr camera_info_client;

    bool device_type_init_;
    bool camera_info_init_;
    std::string device_type_;
    sensor_msgs::msg::CameraInfo camera_info_;
    int uvc_flip_;
    OB_DEVICE_NO device_type_no_;
    bool camera_info_valid_;
    
};
}