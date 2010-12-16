find_package(PkgConfig REQUIRED)
pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

add_library(kinect-render SHARED 
    kinect_renderer.c
    jpeg-utils-ijg.c
    )

add_executable(kinect-viewer main.c)

target_link_libraries(kinect-viewer 
    kinect-render
    jpeg
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(kinect-viewer
    glib-2.0
    bot2-vis
    lcm)

pods_install_libraries(kinect-render)

pods_install_executables(kinect-viewer)
