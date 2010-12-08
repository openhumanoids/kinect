add_definitions(-Wall)

add_executable(freenect-lcm main.c timestamp.c)

target_link_libraries(freenect-lcm 
    freenect
    ${LCMTYPES_LIBS})
pods_use_pkg_config_packages(freenect-lcm
    libusb-1.0
    glib-2.0
    lcm)

#add_executable(stest glview.c)
#target_link_libraries(stest freenect m ${LCMTYPES_LIBS})
#pods_use_pkg_config_packages(stest 
#    libusb-1.0
#    glib-2.0
#    lcm)

pods_install_executables(freenect-lcm)
