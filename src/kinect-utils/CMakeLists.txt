add_definitions(-Wall)

add_library(kinect-utils SHARED kinect-calib.c)

set_target_properties(kinect-utils PROPERTIES SOVERSION 1)

pods_install_headers(kinect-calib.h 
    kinect-utils.h
    DESTINATION kinect)

pods_install_libraries(kinect-utils)

pods_install_pkg_config_file(kinect-utils
    LIBS -lkinect-utils
    VERSION 0.0.1)

add_executable(kinect-pc kinect-pc.cpp kinect-pointcloud-pub.cpp kinect-calib.c)

pods_use_pkg_config_packages(kinect-pc
    glib-2.0
    bot2-vis
    bot2-frames
    lcm bot2-core lcmtypes_kinect )

target_link_libraries(kinect-pc z m ${LCMTYPES_LIBS} ncurses boost_program_options)

#pods_use_pkg_config_packages(lidar-tilt lcm bot2-core lcmtypes_mapdb lcmtypes_visualization lcmtypes_microstrain_comm glib-2.0 lcmtypes_gui )

pods_install_executables(kinect-pc)