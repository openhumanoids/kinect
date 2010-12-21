add_definitions(-Wall)

add_library(kinect-utils SHARED kinect_calib.c)

set_target_properties(kinect-utils PROPERTIES SOVERSION 1)

pods_install_headers(kinect_calib.h DESTINATION kinect)

pods_install_libraries(kinect-utils)

pods_install_pkg_config_file(kinect-utils
    LIBS -lkinect-utils -llcmtypes_kinect
    VERSION 0.0.1)
