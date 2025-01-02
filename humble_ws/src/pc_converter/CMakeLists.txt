cmake_minimum_required(VERSION 3.5)
project(pc_converter CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# add executable and link libraries
add_executable(pc_converter src/converter.cpp)
target_link_libraries(pc_converter ${PCL_LIBRARIES})
ament_target_dependencies(pc_converter
  rclcpp
  sensor_msgs
  pcl_conversions
  )

install(TARGETS
  pc_converter
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()