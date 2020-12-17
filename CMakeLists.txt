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

option(FILTER "OB Filter library" ON)

if( ${FILTER} STREQUAL "ON" )
set(obfilter "On")
else()
set(obfilter "Off")
endif()

include(ExternalProject)

if(WIN32)
  set(patch_exe patch.exe)
else()
  set(patch_exe patch)
endif()

ExternalProject_Add(astra_openni2
  PREFIX astra_openni2
  # SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/astra_openni2/OpenNI2
  GIT_REPOSITORY https://github.com/orbbec/OpenNI2.git
  GIT_TAG orbbec_ros
  GIT_CONFIG user.email=you@example.com user.name=YourName
  CONFIGURE_COMMAND echo "no need to configure"
  #${CMAKE_CURRENT_SOURCE_DIR}/libantlr/configure --prefix=<INSTALL_DIR>
  BUILD_IN_SOURCE 1
  BUILD_COMMAND make release FILTER=${obfilter} CFLAGS+=-fPIC CFLAGS+=-Wno-stringop-truncation CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/ThirdParty/PSCommon/XnLib/Include CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Include CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/Drivers/OniFile/Formats CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/Drivers/OniFile CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/Drivers/orbbec/Include CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/Drivers/orbbec/Core CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/Drivers/orbbec CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Source/DepthUtils CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/ThirdParty/Orbbec/Include CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Samples/Common CFLAGS+=-I${CMAKE_CURRENT_BINARY_DIR}/astra_openni2/src/astra_openni2/Samples/MWClosestPoint  INSTALL_DIR openni2
  INSTALL_COMMAND tar -xjf <SOURCE_DIR>/Packaging/Final/OpenNI-Linux-2.3.tar.bz2 -C <INSTALL_DIR> --strip 1 && mkdir -p <INSTALL_DIR>/include && ln -fs <INSTALL_DIR>/Include <INSTALL_DIR>/include/openni2
  PATCH_COMMAND
    ${patch_exe} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/astra_openni2-patch.diff

)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/openni2/Redist)
include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/openni2/include
                    )

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
target_link_libraries(astra_wrapper -lOpenNI2Orbbec ${Boost_LIBRARIES})
add_dependencies(astra_wrapper astra_openni2)

add_library(astra_driver_lib src/astra_driver.cpp)
ament_target_dependencies(astra_driver_lib
  "builtin_interfaces"
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(astra_driver_lib astra_wrapper ${Boost_LIBRARIES})
add_dependencies(astra_driver_lib astra_openni2)

add_library(astra_camera_nodelet SHARED
  ros/astra_camera_nodelet.cpp
)
target_link_libraries(astra_camera_nodelet astra_driver_lib)
add_dependencies(astra_camera_nodelet astra_openni2)
rclcpp_components_register_nodes(astra_camera_nodelet "astra_camera::AstraDriverNode")

add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
target_link_libraries(astra_camera_node astra_driver_lib)
add_dependencies(astra_camera_node astra_openni2)

find_package(libuvc REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

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

install(FILES ${CMAKE_BINARY_DIR}/openni2/Redist/libOpenNI2Orbbec.so
  DESTINATION lib
)

install(DIRECTORY ${CMAKE_BINARY_DIR}/openni2/Redist/OpenNI2
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
