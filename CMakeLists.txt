cmake_minimum_required(VERSION 2.8.3)
project(hanging_points_generator)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros)

find_package(PCL REQUIRED COMPONENTS
  common
  features
  io
  kdtree
  search
  )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(marching_cubes src/marching_cubes.cpp)
target_link_libraries(marching_cubes
  ${OPENNI_LIBRARIES}
  ${PCL_LIBRARIES}
  pcl_common
  pcl_features
  pcl_io
  pcl_surface
)
