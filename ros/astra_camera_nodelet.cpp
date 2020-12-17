/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2016, Orbbec Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Tim Liu (liuhua@orbbec.com)
 */

#include "astra_camera/astra_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;
namespace astra_camera{
class AstraDriverNode : public rclcpp::Node
{
public:
  AstraDriverNode(const rclcpp::NodeOptions& options) : rclcpp::Node("astra_driver", options)
  {
    // RGB
    size_t width = 1280;
    size_t height = 1024;
    double framerate = 30;

    // Depth
    size_t dwidth = 640;
    size_t dheight = 480;
    double dframerate = 30;
    astra_wrapper::PixelFormat dformat = astra_wrapper::PixelFormat::PIXEL_FORMAT_DEPTH_1_MM;
    
    bool use_ir = true;
    bool use_color = true;
    bool use_depth = true;

    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("astra_camera");
    rclcpp::Node::SharedPtr pnh = rclcpp::Node::make_shared("astra_camera_");

    try
    {
      pnh->declare_parameter("use_ir", use_ir);
      pnh->declare_parameter("use_color", use_color);
      pnh->declare_parameter("use_depth", use_depth);
    }
    catch(rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
      // do nothing
    }
    RCLCPP_INFO(nh->get_logger(), "Initializing...");
    astra_wrapper::AstraDriver drv(nh, pnh, width, height, framerate, dwidth, dheight, dframerate, dformat);
    RCLCPP_INFO(nh->get_logger(), "Initialized");
    rclcpp::spin(nh);
  };

  ~AstraDriverNode() {}
};
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::AstraDriverNode)