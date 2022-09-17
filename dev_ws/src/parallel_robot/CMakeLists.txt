cmake_minimum_required(VERSION 3.8) 
project(parallel_robot) 


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C11 
#if(NOT CMAKE_C_STANDARD) 
  #set(CMAKE_C_STANDARD 11) 
#endif() 

# Default to C++14 

if(NOT CMAKE_CXX_STANDARD) 
  set(CMAKE_CXX_STANDARD 14) 
endif() 

if(NOT WIN32) 
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif() 

# find dependencies 

find_package(ament_cmake REQUIRED) 
find_package(control_msgs REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(rclcpp_action REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(ament_cmake_python REQUIRED) 
find_package(rclpy REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(my_messages REQUIRED)




if(BUILD_TESTING) 
  find_package(ament_cmake_gtest REQUIRED) 
  find_package(ament_lint_auto REQUIRED) 
  ament_lint_auto_find_test_dependencies() 
endif() 


# Include cpp "include" directory 

include_directories(include) 

add_executable(global_parameter_server src/global_parameter_server.cpp) 
ament_target_dependencies(global_parameter_server rclcpp) 

install(TARGETS 
  global_parameter_server 
  DESTINATION lib/${PROJECT_NAME} 
) 

install(DIRECTORY 
  launch meshes urdf config rviz control 
  DESTINATION share/${PROJECT_NAME}/ 
) 



add_executable(control_pid src/control_pid.cpp) 
ament_target_dependencies(control_pid 
  rclcpp 
  rclcpp_action 
  control_msgs
  std_msgs
  my_messages 
  sensor_msgs
) 
add_executable(traject_circular src/traject_circular.cpp) 
ament_target_dependencies(traject_circular
  rclcpp 
  rclcpp_action 
  control_msgs
  std_msgs
  my_messages 
  sensor_msgs
) 
add_executable(traject_rombo src/traject_rombo.cpp) 
ament_target_dependencies(traject_rombo
  rclcpp 
  rclcpp_action 
  control_msgs
  std_msgs
  my_messages 
  sensor_msgs
) 
add_executable(traject_triangulo src/traject_triangulo.cpp) 
ament_target_dependencies(traject_triangulo
  rclcpp 
  rclcpp_action 
  control_msgs
  std_msgs
  my_messages 
  sensor_msgs
) 

#add_executable(publicador src/publicador.cpp) 
#ament_target_dependencies( publicador
 # rclcpp 
 # std_msgs
 # my_messages
#) 
add_executable(suscriptor src/suscriptor.cpp) 
ament_target_dependencies( suscriptor
  rclcpp 
  std_msgs
  sensor_msgs
  my_messages
) 


# Install Python executables
install(PROGRAMS
  scripts/posicion.py
  DESTINATION lib/${PROJECT_NAME}
)
#lineas anadidas
install(TARGETS 
 
  suscriptor 
  DESTINATION lib/${PROJECT_NAME} 
) 
install(TARGETS 
 # publicador
  control_pid
  traject_circular
  traject_triangulo
  traject_rombo
  DESTINATION lib/${PROJECT_NAME} 
) 


ament_package() 





