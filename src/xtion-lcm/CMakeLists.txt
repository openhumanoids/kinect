include(../../cmake/sse.cmake)
if (NOT SSE3_FOUND)
	message(STATUS, "sse3 not found, not building xtion-lcm")
	return()
endif()
add_definitions(-Wall -std=gnu99 -msse2 -msse3 -DHAVE_INTEL)

add_executable(xtion-lcm main.cpp
    timestamp.cpp
    pixels.c
    pixels_sse2.c
    pixels_sse3.c
    cpuid.c
    jpeg-utils-ijg.c)

#add_executable(test-sender test_sender.c timestamp.cpp)

target_link_libraries(xtion-lcm 
    #glib-2.0
    gthread-2.0
    freenect
    jpeg
    z
    m
    boost_program_options-mt
    ${LCMTYPES_LIBS})

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(xtion-lcm pcl_common pcl_io pcl_visualization) 

#include_directories(/usr/include/pcl-1.5)
#include_directories(/usr/include/openni)
#include_directories(/usr/include/libusb-1.0)
#include_directories(/usr/include/eigen3)

#target_link_libraries(test-sender
#    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(xtion-lcm
    glib-2.0
    libusb-1.0
    lcm
    #    jpeg-utils
    )

#pods_use_pkg_config_packages(test-sender
#    lcm)

pods_install_executables(xtion-lcm)
#pods_install_executables(test-sender)
