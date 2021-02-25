#include "libuvc_camera/camera_driver.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <libuvc/libuvc.h>
#include <cmath>
#include "astra_camera/ros12_shim.h"
#include <chrono>
using namespace std::chrono_literals;

namespace libuvc_camera {
CameraDriver::CameraDriver()
    : rclcpp::Node("libuvc_camera"),
      state_(kInitial),
      ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
      cinfo_manager_(this),
      param_init_(false)
{
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    ns = get_namespace();
    RCLCPP_INFO(get_logger(), "Creating clients and services...");
    device_type_client = create_client<astra_camera::srv::GetDeviceType>(ns + "/get_device_type");
    camera_info_client = create_client<astra_camera::srv::GetCameraInfo>(ns + "/get_camera_info");
    get_uvc_exposure_server = create_service<astra_camera::srv::GetUVCExposure>("get_uvc_exposure", 
                                std::bind(&CameraDriver::getUVCExposureCb, this, std::placeholders::_1, std::placeholders::_2));
    set_uvc_exposure_server = create_service<astra_camera::srv::SetUVCExposure>("set_uvc_exposure", 
                                std::bind(&CameraDriver::setUVCExposureCb, this, std::placeholders::_1, std::placeholders::_2));
    get_uvc_gain_server = create_service<astra_camera::srv::GetUVCGain>("get_uvc_gain", 
                                std::bind(&CameraDriver::getUVCGainCb, this, std::placeholders::_1, std::placeholders::_2));
    set_uvc_gain_server = create_service<astra_camera::srv::SetUVCGain>("set_uvc_gain", 
                                std::bind(&CameraDriver::setUVCGainCb, this, std::placeholders::_1, std::placeholders::_2));
    get_uvc_white_balance_server = create_service<astra_camera::srv::GetUVCWhiteBalance>("get_uvc_white_balance", 
                                std::bind(&CameraDriver::getUVCWhiteBalanceCb, this, std::placeholders::_1, std::placeholders::_2));
    set_uvc_white_balance_server = create_service<astra_camera::srv::SetUVCWhiteBalance>("set_uvc_white_balance", 
                                std::bind(&CameraDriver::setUVCWhiteBalanceCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Clients and services ready");
    device_type_init_ = false;
    camera_info_init_ = false;
    uvc_flip_ = 0;
    device_type_no_ = OB_ASTRA_NO;
    int slash_end;
    for(slash_end = 0; slash_end < ns.length(); slash_end++)
    {
        if(ns[slash_end] != '/') break;
    }
    ns_no_slash = ns.substr(slash_end);
    camera_info_valid_ = false;
    // declare parameters
    declare_parameter("pan_absolute");
    declare_parameter("tilt_absolute");
    declare_parameter("vendor");
    declare_parameter("product");
    declare_parameter("serial");
    declare_parameter("index");
    declare_parameter("width");
    declare_parameter("height");
    declare_parameter("video_mode");
    declare_parameter("frame_rate");
    declare_parameter("frame_id");
}

CameraDriver::~CameraDriver()
{
    if(rgb_frame_) {
        uvc_free_frame(rgb_frame_);
    }
    if(ctx_) {
        uvc_exit(ctx_);
    }
}

bool CameraDriver::getUVCExposureCb(const std::shared_ptr<astra_camera::srv::GetUVCExposure::Request> request, std::shared_ptr<astra_camera::srv::GetUVCExposure::Response> response)
{
    uint32_t expo;
    uvc_error_t err = uvc_get_exposure_abs(devh_, &expo, UVC_GET_CUR);
    response->exposure = expo;
    return (err == UVC_SUCCESS);
}

bool CameraDriver::setUVCExposureCb(const std::shared_ptr<astra_camera::srv::SetUVCExposure::Request> request, std::shared_ptr<astra_camera::srv::SetUVCExposure::Response> response)
{
    if(request->exposure == 0)
    {
        uvc_set_ae_mode(devh_, 2);
        return true;
    }
    uvc_set_ae_mode(devh_, 1);
    if(request->exposure > 330)
    {
        ROS_ERROR("Please set exposure lower than 330");
        return false;
    }
    uvc_error_t err = uvc_set_exposure_abs(devh_, request->exposure);
    return (err == UVC_SUCCESS);
}

bool CameraDriver::getUVCGainCb(const std::shared_ptr<astra_camera::srv::GetUVCGain::Request> request, std::shared_ptr<astra_camera::srv::GetUVCGain::Response> response)
{
    uint16_t gain;
    uvc_error_t err = uvc_get_gain(devh_, &gain, UVC_GET_CUR);
    response->gain = gain;
    return (err == UVC_SUCCESS);
}

bool CameraDriver::setUVCGainCb(const std::shared_ptr<astra_camera::srv::SetUVCGain::Request> request, std::shared_ptr<astra_camera::srv::SetUVCGain::Response> response)
{
    uvc_error_t err = uvc_set_gain(devh_, request->gain);
    return (err == UVC_SUCCESS);
}

bool CameraDriver::getUVCWhiteBalanceCb(const std::shared_ptr<astra_camera::srv::GetUVCWhiteBalance::Request> request, std::shared_ptr<astra_camera::srv::GetUVCWhiteBalance::Response> response)
{
    uint16_t white_balance;
    uvc_error_t err = uvc_get_white_balance_temperature(devh_, &white_balance, UVC_GET_CUR);
    response->white_balance = white_balance;
    return (err == UVC_SUCCESS);
}

bool CameraDriver::setUVCWhiteBalanceCb(const std::shared_ptr<astra_camera::srv::SetUVCWhiteBalance::Request> request, std::shared_ptr<astra_camera::srv::SetUVCWhiteBalance::Response> response)
{
    if(request->white_balance == 0)
    {
        uvc_set_white_balance_temperature_auto(devh_, 1);
        return true;
    }
    uvc_set_white_balance_temperature_auto(devh_, 0);
    uvc_error_t err = uvc_set_white_balance_temperature(devh_, request->white_balance);
    return (err == UVC_SUCCESS);
}

bool CameraDriver::Start()
{
    RCLCPP_INFO(get_logger(), "Libuvc camera starting...");
    assert(state_ == kInitial);
    uvc_error_t err = uvc_init(&ctx_, NULL);
    if(err != UVC_SUCCESS)
    {
        uvc_perror(err, "ERROR: uvc_init");
        return false;
    }
    state_ = kStopped;
    param_client_->on_parameter_event(std::bind(&CameraDriver::ReconfigureCallback, this, std::placeholders::_1));
    OpenCamera();
    RCLCPP_INFO(get_logger(), "Parameter event callback registered, return value: %d", state_ == kRunning);
    return state_ == kRunning;
}

void CameraDriver::Stop()
{
    boost::recursive_mutex::scoped_lock(mutex_);

    assert(state_ != kInitial);
    if(state_ == kRunning) {
        CloseCamera();
    }
    assert(state_ == kStopped);
    uvc_exit(ctx_);
    ctx_ = NULL;
    state_ = kInitial;
}

void CameraDriver::ReconfigureCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    RCLCPP_INFO(get_logger(), "On parameter event");
    boost::recursive_mutex::scoped_lock(mutex_);
    const std::string full_name = std::string(get_namespace()) + std::string(get_name());
    if(event->node != full_name) {
        return;
    }

    if(state_ == kStopped) {
        OpenCamera();
    }
    
    for(auto param : event->changed_parameters)
    {
        if(param.name == "camera_info_url") {
            cinfo_manager_.loadCameraInfo(param.value.string_value);
            break;
        }
    }
    if (state_ == kRunning) {
#define PARAM_INT(nm, fn, value) if (param.name == #nm) { \
      int val = (value);                                                \
      if (uvc_set_##fn(devh_, val)) {                                   \
        ROS_WARN("Unable to set " #nm " to %d", val);                 \
      }                                                                 \
    }
    for(auto param : event->changed_parameters) {
        PARAM_INT(scanning_mode, scanning_mode, param.value.integer_value);
        PARAM_INT(auto_exposure, ae_mode, 1 << param.value.integer_value);
        PARAM_INT(auto_exposure_priority, ae_priority, param.value.integer_value);
        PARAM_INT(exposure_absolute, exposure_abs, param.value.double_value * 10000);
        PARAM_INT(auto_focus, focus_auto, param.value.bool_value ? 1 : 0);
        PARAM_INT(focus_absolute, focus_abs, param.value.integer_value);
#if LIBUVC_VERSION_GTE(0,0,6)
    PARAM_INT(gain, gain, param.value.integer_value);
    PARAM_INT(iris_absolute, iris_abs, param.value.integer_value);
    PARAM_INT(brightness, brightness, param.value.integer_value);
#endif
        if(param.name == "pan_absolute" || param.name == "tilt_absolute") {
            auto pan_abs = get_parameter("pan_absolute").as_int();
            auto tilt_abs = get_parameter("tilt_absolute").as_int();
            if(uvc_set_pantilt_abs(devh_, pan_abs, tilt_abs)) {
                ROS_WARN("Unable to set pantilt to %d, %d", pan_abs, tilt_abs);
            }
        }
    }
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: contrast
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: saturation
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }
}

void CameraDriver::CloseCamera()
{
    assert(state_ == kRunning);

    uvc_close(devh_);
    devh_ = NULL;
    
    uvc_unref_device(dev_);
    dev_ = NULL;

    state_ = kStopped;
}

void CameraDriver::OpenCamera()
{
    assert(state_ == kStopped);

    int vendor_id = strtol(get_parameter("vendor").value_to_string().c_str(), NULL, 0);
    int product_id = strtol(get_parameter("product").value_to_string().c_str(), NULL, 0);
    std::string serial;
    get_parameter_or("serial", serial, std::string(""));
    auto index = get_parameter("index").as_int();
    ROS_INFO("Openning camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
             vendor_id, product_id, serial.c_str(), index);

    uvc_device_t **devs;
#if LIBUVC_VERSION_GTE(0,0,6)
    uvc_error_t find_err = uvc_find_devices(
        ctx_, &devs,
        vendor_id,
        product_id,
        serial.empty() ? NULL : serial.c_str());
    if(find_err != UVC_SUCCESS) {
        uvc_perror(find_err, "uvc_find_device");
        return;
    }

    dev_ = NULL;
    int dev_idx = 0;
    while(devs[dev_idx] != NULL) {
        if(dev_idx == index) {
            dev_ = devs[dev_idx];
        }
        else {
            uvc_unref_device(devs[dev_idx]);
        }
        dev_idx++;
    }

    if(dev_ == NULL) {
        ROS_ERROR("Unable to find device at index %d", index);
        return;
    }
#else
    uvc_error_t find_err = uvc_find_device(
        ctx_, &dev_,
        vendor_id,
        product_id,
        serial.empty() ? NULL : serial.c_str());
    if(find_err != UVC_SUCCESS) {
        uvc_perror(find_err, "uvc_find_device");
        return;
    }
#endif
    uvc_error_t open_err = uvc_open(dev_, &devh_);
    if(open_err != UVC_SUCCESS) {
        switch(open_err) {
        case UVC_ERROR_ACCESS:
#ifdef __linux__
            ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                      uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#else
            ROS_ERROR("Permission denied opening device %d on bus %d",
                      uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
            break;
        default:
#ifdef __linux__
            ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                      uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                      uvc_strerror(open_err), open_err);
#else
            ROS_ERROR("can't open device %d on bus %d: %s (%d)",
                      uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                      uvc_strerror(open_err), open_err);
#endif
            break;
        }
        uvc_unref_device(dev_);
        return;
    }

    uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);

    uvc_stream_ctrl_t ctrl;
    uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
        devh_, &ctrl,
        GetVideoMode(get_parameter("video_mode").as_string()),
        get_parameter("width").as_int(), get_parameter("height").as_int(),
        get_parameter("frame_rate").as_int());
    
    if(mode_err != UVC_SUCCESS) {
        uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh_);
        uvc_unref_device(dev_);
        ROS_ERROR("check video_mode/width/height/frame_rate are available");
        uvc_print_diag(devh_, NULL);
        return;
    }

    uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this, 0);
    if(stream_err != UVC_SUCCESS) {
        uvc_perror(stream_err, "uvc_start_streaming");
        uvc_close(devh_);
        uvc_unref_device(dev_);
        return;
    }

    if(rgb_frame_) {
        uvc_free_frame(rgb_frame_);
    }

    rgb_frame_ = uvc_allocate_frame(get_parameter("width").as_int() * get_parameter("height").as_int() * 3);
    assert(rgb_frame_);

    state_ = kRunning;
}

void CameraDriver::AutoControlsCallbackAdapter(
    enum uvc_status_class status_class,
    int event,
    int selector,
    enum uvc_status_attribute status_attribute,
    void *data, size_t data_len,
    void *ptr)
{
    CameraDriver *driver = static_cast<CameraDriver*>(ptr);
    driver->AutoControlsCallback(status_class, event, selector, status_attribute, data, data_len);
}

void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr)
{
    CameraDriver *driver = static_cast<CameraDriver*>(ptr);
    driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(
    enum uvc_status_class status_class,
    int event,
    int selector,
    enum uvc_status_attribute status_attribute,
    void *data, size_t data_len)
{
    boost::recursive_mutex::scoped_lock(mutex_);

    printf("Controls callback. class: %d, event: 5d, selector: %d, attr: %d, data_len: %zu\n",
           status_class, event, selector, status_attribute, data_len);
    
    if(status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
        switch(status_class) {
        case UVC_STATUS_CLASS_CONTROL_CAMERA: {
            switch(selector) {
            case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
                uint8_t *data_char = (uint8_t*) data;
                uint32_t exposure_int = ((data_char[0]) | (data_char[1] << 8) |
                                         (data_char[2] << 16) | (data_char[3] << 24));
                set_parameter(rclcpp::Parameter("exposure_absolute", exposure_int * 0.0001));
                break;
            }
            break;
        }
        case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
            switch(selector) {
            case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
                uint8_t *data_char = (uint8_t*) data;
                set_parameter(rclcpp::Parameter("white_balance_temperature", data_char[0] | (data_char[1] << 8)));
                break;
            }
            break;
        }
        }
    }
}

void CameraDriver::ImageCallback(uvc_frame_t *frame)
{
    rclcpp::Time timestamp(frame->capture_time.tv_sec, frame->capture_time.tv_usec);
    if(timestamp == rclcpp::Time(0)) {
        timestamp = now();
    }
    boost::recursive_mutex::scoped_lock(mutex_);

    assert(state_ == kRunning);
    assert(rgb_frame_);

    sensor_msgs::msg::Image::Ptr image(new sensor_msgs::msg::Image());
    image->width = get_parameter("width").as_int();
    image->height = get_parameter("height").as_int();
    image->step = image->width * 3;
    image->data.resize(image->step * image->height);

    if(frame->frame_format == UVC_FRAME_FORMAT_BGR) {
        image->encoding = "bgr8";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
    } else if(frame->frame_format == UVC_FRAME_FORMAT_RGB) {
        image->encoding = "rgb8";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
    } else if(frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
        image->encoding = "yuv422";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
    } else if(frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
        // FIXME: uvc_any2bgr does not work on "yuyv" format, so use uvc_yuyv2bgr directly.
        uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
        if(conv_ret != UVC_SUCCESS) {
            uvc_perror(conv_ret, "Couldn't convert frame to RGB");
            return;
        }
        image->encoding = "bgr8";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
#if LIBUVC_VERSION_GTE(0,0,6)
    } else if(frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
        uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
        if(conv_ret != UVC_SUCCESS) {
            uvc_perror(conv_ret, "Couldn't cconvert frame to RGB");
            return;
        }
        image->encoding = "rgb8";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
#endif
    } else {
        uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
        if(conv_ret != UVC_SUCCESS) {
            uvc_perror(conv_ret, "Couldn't convert frame to RGB");
            return;
        }
        image->encoding = "bgr8";
        memcpy(&(image->data[0]), frame->data, frame->data_bytes);
    }
    
    auto device_type_srv = std::make_shared<astra_camera::srv::GetDeviceType::Request>();
    auto camera_info_srv = std::make_shared<astra_camera::srv::GetCameraInfo::Request>();

    if(device_type_init_ == false)
    {
        // wait for service ready
        while (!device_type_client->wait_for_service(1s))
        {
            if(!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }
        auto result = device_type_client->async_send_request(device_type_srv);
        auto status = result.wait_for(1s);
        while(status != std::future_status::ready) {
            status = result.wait_for(1s);
        }        
	device_type_ = result.get()->device_type;
        if(strcmp(device_type_.c_str(), OB_STEREO_S) == 0) {
            device_type_no_ = OB_STEREO_S_NO;
        } else if (strcmp(device_type_.c_str(), OB_EMBEDDED_S) == 0) {
            device_type_no_ = OB_EMBEDDED_S_NO;
        } else if (strcmp(device_type_.c_str(), OB_ASTRA_PRO) == 0) {
            device_type_no_ = OB_ASTRA_PRO_NO;
        } else if (strcmp(device_type_.c_str(), OB_STEREO_S_U3) == 0) {
            device_type_no_ = OB_STEREO_S_U3_NO;
        } else {
            device_type_no_ = OB_ASTRA_NO;
        }
        device_type_init_ = true;
    }
    
    if(camera_info_init_ == false)
    {
        // wait for service ready
        while (!camera_info_client->wait_for_service(1s))
        {
            if(!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }
        auto result = camera_info_client->async_send_request(camera_info_srv);
        auto status = result.wait_for(1s);
        while(status != std::future_status::ready) {
            status = result.wait_for(1s);
        }	
	camera_info_ = result.get()->info;
        camera_info_init_ = true;
        camera_info_valid_ = true;
        if(std::isnan(camera_info_.k[0]) || std::isnan(camera_info_.k[2]) || std::isnan(camera_info_.k[4]) || std::isnan(camera_info_.k[5]))
        {
            camera_info_valid_ = false;
        }
    }

    sensor_msgs::msg::CameraInfo::Ptr cinfo(new sensor_msgs::msg::CameraInfo(cinfo_manager_.getCameraInfo()));
    if(device_type_init_ == true && astraWithUVC(device_type_no_))
    {
        if(camera_info_init_ == true && camera_info_valid_ == true)
        {
            cinfo->height = image->height;
            cinfo->width = image->width;
            cinfo->distortion_model = camera_info_.distortion_model;
            cinfo->d.resize(5, 0.0);
            cinfo->d[4] = 0.0000000001;
            for(int i = 0; i < 9; i++)
            {
                cinfo->k[i] = camera_info_.k[i];
                cinfo->r[i] = camera_info_.r[i];
            }
            cinfo->k[0] = (1 - uvc_flip_) * camera_info_.k[0] + uvc_flip_ * (-camera_info_.k[0]);
            cinfo->k[2] = (1 - uvc_flip_) * camera_info_.k[2] + uvc_flip_ * (image->width - camera_info_.k[2]);
            for(int i = 0; i < 12; i++)
            {
                cinfo->p[i] = camera_info_.p[i];
            }
        }
        image->header.frame_id = ns_no_slash + "_rgb_optical_frame";
        cinfo->header.frame_id = ns_no_slash + "_rgb_optical_frame";
    }
    else
    {
        image->header.frame_id = get_parameter("frame_id").as_string();
        cinfo->header.frame_id = image->header.frame_id;
    }
    image->header.stamp = timestamp;
    cinfo->header.stamp = timestamp;
    if(cam_pub_ == nullptr) {
        it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        cam_pub_ = std::make_unique<image_transport::CameraPublisher>(it_->advertiseCamera("image_raw", 1, false));
    }
    cam_pub_->publish(image, cinfo);
}

enum uvc_frame_format CameraDriver::GetVideoMode(std::string vmode)
{
    if(vmode == "uncompressed") {
        return UVC_COLOR_FORMAT_UNCOMPRESSED;
    } else if (vmode == "compressed") {
        return UVC_COLOR_FORMAT_COMPRESSED;
    } else if (vmode == "yuyv") {
        return UVC_COLOR_FORMAT_YUYV;
    } else if (vmode == "uyuv") {
        return UVC_COLOR_FORMAT_UYVY;
    } else if (vmode == "rgb") {
        return UVC_COLOR_FORMAT_RGB;
    } else if (vmode == "bgr") {
        return UVC_COLOR_FORMAT_BGR;
    } else if (vmode == "mjpeg") {
        return UVC_COLOR_FORMAT_MJPEG;
    } else if (vmode == "gray8") {
        return UVC_COLOR_FORMAT_GRAY8;
    } else {
        RCLCPP_ERROR(get_logger(), "Invalid Video Mode: %s", vmode);
        RCLCPP_WARN(get_logger(), "Continue using video mode: uncompressed");
        return UVC_COLOR_FORMAT_UNCOMPRESSED;
    }
}
}
