#include <rclcpp/rclcpp.hpp>
#include "libuvc_camera/camera_driver.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto driver = std::make_shared<libuvc_camera::CameraDriver>();

    if(!driver->Start()) {
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(driver);
    driver->Stop();
    rclcpp::shutdown();
    return 0;
}