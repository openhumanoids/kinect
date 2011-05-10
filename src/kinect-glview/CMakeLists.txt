add_executable(kinect-glview main.c jpeg-utils-ijg.c)

pods_use_pkg_config_packages(kinect-glview 
    lcm
    glib-2.0
    )

target_link_libraries(kinect-glview 
    ${LCMTYPES_LIBS} 
    z 
    jpeg
    )

if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
else()
  find_package(OpenGL REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
  target_link_libraries(kinect-glview ${OPENGL_LIBRARIES} glut m)
endif()

pods_install_executables(kinect-glview)
