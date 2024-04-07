cmake_minimum_required(VERSION 3.8)
project(leptrino_force_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(leptrino_force_sensor 
src/leptrino_node.cpp
src/rs_comm.cpp
)

ament_target_dependencies(leptrino_force_sensor rclcpp geometry_msgs)

include_directories(include)


target_include_directories(leptrino_force_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(leptrino_force_sensor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  leptrino_force_sensor
  DESTINATION lib/${PROJECT_NAME}
  )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
