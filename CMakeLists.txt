cmake_minimum_required(VERSION 3.5)
project(astra_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -fPIC -fmax-errors=2)
  add_compile_options(-Wno-deprecated)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rclcpp_components REQUIRED)

#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m  OUTPUT_VARIABLE  MACHINES )
execute_process(COMMAND getconf LONG_BIT  OUTPUT_VARIABLE  MACHINES_BIT )
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
IF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "x64")
# for schroot enviroment
ELSEIF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "x86" )
ELSEIF ( ${MACHINES} MATCHES "x86" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i686" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i386" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "arm" )
      set(HOST_PLATFORM  "arm")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "arm64")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "arm")
ENDIF ()

message(STATUS "ORRBEC : ${HOST_PLATFORM}")
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

set(ORBBEC_OPENNI2_REDIST  "${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/${HOST_PLATFORM}")
link_directories(${ORBBEC_OPENNI2_REDIST})
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2 
)

find_package(libuvc REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(${libuvc_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/GetDeviceType.srv
  srv/GetCameraInfo.srv
  srv/GetUVCExposure.srv
  srv/GetUVCGain.srv
  srv/GetUVCWhiteBalance.srv
  srv/SetUVCExposure.srv
  srv/SetUVCGain.srv
  srv/SetUVCWhiteBalance.srv
  DEPENDENCIES builtin_interfaces sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
)
ament_target_dependencies(astra_wrapper
  "builtin_interfaces"
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(astra_wrapper -lOpenNI2  -L${ORBBEC_OPENNI2_REDIST} ${Boost_LIBRARIES})

add_library(astra_driver_lib src/astra_driver.cpp)
rosidl_target_interfaces(astra_driver_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(astra_driver_lib
  "builtin_interfaces"
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(astra_driver_lib astra_wrapper ${Boost_LIBRARIES})

add_library(astra_camera_nodelet SHARED
  ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib ${Boost_LIBRARIES})
rclcpp_components_register_nodes(astra_camera_nodelet "astra_camera::AstraDriverNode")

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib ${Boost_LIBRARIES})

message(STATUS "libuvc  ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")
add_executable(camera_node 
  src/libuvc_camera/main.cpp
  src/libuvc_camera/camera_driver.cpp
  src/astra_device_type.cpp
)
rosidl_target_interfaces(camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  image_transport
)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS astra_camera_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS astra_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS astra_wrapper astra_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES include/openni2_redist/${HOST_PLATFORM}/libOpenNI2.so
  DESTINATION lib
)

install(DIRECTORY include/openni2_redist/${HOST_PLATFORM}/OpenNI2
  DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(FILES 56-orbbec-usb.rules
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/create_udev_rules
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(astra_driver_lib astra_wrapper)
ament_package()
