add_definitions(-Wall)

add_executable(kinect-lcm main.c timestamp.c)

target_link_libraries(kinect-lcm 
    freenect
    ${LCMTYPES_LIBS})
pods_use_pkg_config_packages(kinect-lcm
    libusb-1.0
    glib-2.0
    lcm)

pods_install_executables(kinect-lcm)
