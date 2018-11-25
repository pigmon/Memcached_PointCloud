cmake_minimum_required(VERSION 2.8.3)
project(read_memc)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mesh_marker
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_libraries(memcached)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#############
## Install ##
#############


#############
## Testing ##
#############

add_executable(pc_read 
	include/mc_reader.h src/mc_reader.cpp
	src/main.cpp)
target_link_libraries(pc_read ${catkin_LIBRARIES})
