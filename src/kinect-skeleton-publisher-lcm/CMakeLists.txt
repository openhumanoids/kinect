cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME kinect-skeleton-publisher-lcm)
include(cmake/pods.cmake)

include_directories(. 
  ../../openni/openni/build/openni/Include
  ../../openni/ps_engine/build/ps_engine/Include
  ../../openni/nite/build/Nite-1.3.1.5/Include
  ../../../../core/path-util/src/
)#/usr/include/ni)

# create an executable, and make it public
add_executable(kinect-skeleton-publisher-lcm 
  src/kinect_skeleton_publisher_openni.cc)

target_link_libraries(kinect-skeleton-publisher-lcm 
  /home/spillai/envoy_latest/software/drivers/kinect/openni/openni/build/openni/Platform/Linux-x86/Redist/Lib/libOpenNI.so z m ${LCMTYPES_LIBS} ncurses boost_program_options boost_thread usb-1.0)

pods_use_pkg_config_packages ( kinect-skeleton-publisher-lcm lcm bot2-core glib-2.0 er-path-util)

pods_install_executables(kinect-skeleton-publisher-lcm)
