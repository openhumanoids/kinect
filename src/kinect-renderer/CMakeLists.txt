add_definitions(-Wall -std=gnu99)

add_library(kinect-render SHARED 
    kinect_renderer.c)

add_executable(kinect-viewer main.c)

target_link_libraries(kinect-viewer 
    kinect-render
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(kinect-viewer
    glib-2.0
    bot2-vis
    lcm)

pods_install_libraries(kinect-render)

pods_install_executables(kinect-viewer)
