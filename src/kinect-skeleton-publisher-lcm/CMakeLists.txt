cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME kinect-skeleton-publisher-lcm)
include(cmake/pods.cmake)

include_directories(. /home/spillai/envoy_latest/software/build/include/ni)#/usr/include/ni)

# create an executable, and make it public
add_executable(kinect-skeleton-publisher-lcm 
  src/kinect_skeleton_publisher_openni.cc)

target_link_libraries(kinect-skeleton-publisher-lcm -L/home/spillai/envoy_latest/software/build/lib/ OpenNI z m ${LCMTYPES_LIBS} ncurses boost_program_options boost_thread usb-1.0)

pods_use_pkg_config_packages ( kinect-skeleton-publisher-lcm lcm bot2-core glib-2.0)

pods_install_executables(kinect-skeleton-publisher-lcm)
