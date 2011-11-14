add_definitions(-Wall)

set(CMAKE_CXX_FLAGS "-g")

#include_directories(/opt/ros/electric/stacks/openni_kinect/openni_camera/include /usr/include/openni)
include_directories(. openni_camera /home/sachih/software/envoy/software/build/include/ni)#/usr/include/ni)
#link_directories(/opt/ros/electric/stacks/openni_kinect/openni_camera/lib)

add_executable(kinect-openni-lcm 
  main.cpp 
  kinectOpenniLCM.cpp 
  openni_camera/openni_driver.cpp
  openni_camera/openni_device.cpp
  openni_camera/openni_exception.cpp
  openni_camera/openni_device_primesense.cpp
  openni_camera/openni_device_kinect.cpp
  openni_camera/openni_device_xtion.cpp
  openni_camera/openni_device_oni.cpp
  openni_camera/openni_image_yuv_422.cpp
  openni_camera/openni_image_bayer_grbg.cpp
  openni_camera/openni_image_rgb24.cpp
  openni_camera/openni_ir_image.cpp
  openni_camera/openni_depth_image.cpp
)

target_link_libraries(kinect-openni-lcm -L/home/sachih/software/envoy/software/build/lib/ OpenNI z m ${LCMTYPES_LIBS} ncurses boost_program_options boost_thread usb-1.0)

pods_use_pkg_config_packages ( kinect-openni-lcm lcm bot2-core glib-2.0)

pods_install_executables(kinect-openni-lcm)


