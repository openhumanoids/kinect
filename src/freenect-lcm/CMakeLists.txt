add_executable(freenect-lcm main.c)

target_link_libraries(freenect-lcm 
    freenect
    ${LCMTYPES_LIBS})
pods_use_pkg_config_packages(freenect-lcm
    libusb-1.0
    glib-2.0
    lcm)

pods_install_executables(freenect-lcm)
