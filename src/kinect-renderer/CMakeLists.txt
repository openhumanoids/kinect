add_definitions(-Wall)

add_library(kinect-render SHARED kinect_renderer.c)

#add_executable(kinect-renderer main.c)
#
#target_link_libraries(kinect-renderer kinect-render )
#    freenect
#    ${LCMTYPES_LIBS})
#pods_use_pkg_config_packages(kinect-lcm
#    libusb-1.0
#    glib-2.0
#    lcm)
#
#pods_install_executables(kinect-lcm)
