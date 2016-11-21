find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)


add_definitions(-Wall -std=gnu99)

add_executable(kinect-rgb-tool rgb_tool.cpp)

pods_use_pkg_config_packages(kinect-rgb-tool
    glib-2.0
    lcm
    bot2-core
    kinect-utils)

pods_install_executables(kinect-rgb-tool)
