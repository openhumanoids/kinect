add_definitions(-Wall -std=gnu99 -msse2 -msse3 -DHAVE_INTEL)

add_executable(kinect-lcm main.c 
    timestamp.c 
    pixels.c
    pixels_sse2.c
    pixels_sse3.c
    cpuid.c
    jpeg-utils-ijg.c)

add_executable(test-sender test_sender.c timestamp.c)

target_link_libraries(kinect-lcm 
    freenect
    jpeg
    z
    m
    ${LCMTYPES_LIBS})

target_link_libraries(test-sender
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(kinect-lcm
    libusb-1.0
    glib-2.0
    lcm
    #    jpeg-utils
    )

pods_use_pkg_config_packages(test-sender
    lcm)

pods_install_executables(kinect-lcm)
pods_install_executables(test-sender)
