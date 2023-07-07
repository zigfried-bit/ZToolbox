cmake_minimum_required(VERSION 2.8.3)
project(lidar_height_mes)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

cs_add_executable(heightMesOneframe src/heightMesOneframe.cpp)
target_link_libraries(heightMesOneframe ${PCL_LIBRARIES})

cs_add_executable(lidarHeightMes src/lidarHeightMes.cpp)
target_link_libraries(lidarHeightMes ${PCL_LIBRARIES})
