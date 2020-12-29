cmake_minimum_required(VERSION 2.8.3)
project(glmapping)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED )
find_package (yaml-cpp REQUIRED )
find_package (PCL REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    message_filters
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    tf2
    )

add_message_files(
    FILES
    local2global.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/"
    )

## Declare a C++ library
add_library(glmapping
    src/nodelet_globalmap.cpp
    src/nodelet_localmap.cpp
    src/cylindrical_cell.cpp
    src/local_map_cylindrical.cpp
    src/cartesian_cell.cpp
    src/global_map_cartesian.cpp
    src/rviz_vis.cpp
    src/msg_local2global.cpp
    src/global2occupancygrid2d.cpp
    src/global2esdf.cpp
    src/global2esdf3d.cpp
    )

add_dependencies(glmapping
    glmapping_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(glmapping
    ${catkin_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    )

#independent modules
#1 euroc_publisher publish path
#add_executable(vo_repub_rec
#    src/independ_modules/vo_repub_rec.cpp)
#target_link_libraries(vo_repub_rec
#    ${catkin_LIBRARIES})

#2 output to files
#add_executable(w2files
#    src/independ_modules/w2files.cpp)
#target_link_libraries(w2files
#    ${catkin_LIBRARIES})
