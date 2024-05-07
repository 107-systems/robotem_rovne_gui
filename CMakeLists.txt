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
add_executable(${ROBOTEM_ROVNE_GUI_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_include_directories(${ROBOTEM_ROVNE_GUI_TARGET} PRIVATE
        include
        ${mp_units_INCLUDE_DIRS}
)
#######################################################################################
target_compile_features(${ROBOTEM_ROVNE_GUI_TARGET} PRIVATE cxx_std_20)
#######################################################################################
target_link_libraries(${ROBOTEM_ROVNE_GUI_TARGET}
        ${rclcpp_LIBRARIES}
        ${mp-units_LIBRARIES}
)
#######################################################################################
ament_target_dependencies(${ROBOTEM_ROVNE_GUI_TARGET} rclcpp std_msgs)
#######################################################################################
install(TARGETS ${ROBOTEM_ROVNE_GUI_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#######################################################################################
ament_package()
#######################################################################################
