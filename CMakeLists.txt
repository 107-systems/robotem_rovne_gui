#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(robotem_rovne_gui)
set(ROBOTEM_ROVNE_GUI_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mp-units REQUIRED)
#######################################################################################
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
  message("OpenGL found")
  message ("OPENGL_LIBRARIES: " ${OPENGL_LIBRARIES})
endif()
#######################################################################################
find_package(GLUT REQUIRED 3.4.0)
if(GLUT_FOUND)
  message("GLUT found")
  message ("GLUT_INCLUDE_DIRS: " ${GLUT_INCLUDE_DIRS})
  message ("GLUT_LIBRARIES: " ${GLUT_LIBRARIES})
endif()
#######################################################################################
add_executable(${ROBOTEM_ROVNE_GUI_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_include_directories(${ROBOTEM_ROVNE_GUI_TARGET} PRIVATE
        include
        ${mp_units_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)
#######################################################################################
target_compile_features(${ROBOTEM_ROVNE_GUI_TARGET} PRIVATE cxx_std_20)
#######################################################################################
target_link_libraries(${ROBOTEM_ROVNE_GUI_TARGET}
        ${rclcpp_LIBRARIES}
        ${mp-units_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
)
#######################################################################################
ament_target_dependencies(${ROBOTEM_ROVNE_GUI_TARGET} rclcpp std_msgs)
#######################################################################################
install(TARGETS ${ROBOTEM_ROVNE_GUI_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#######################################################################################
ament_package()
#######################################################################################