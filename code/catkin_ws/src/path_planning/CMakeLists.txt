cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

# Add debug option
set(CMAKE_BUILD_TYPE Debug) # Release or Debug
set(CMAKE_PREFIX_PATH "/opt/ros/noetic" ${CMAKE_PREFIX_PATH})
# set(Python3_DIR "/usr/lib/python3.8/config-3.8-x86_64-linux-gnu")
set(MATPLOTLIBCPP_DIR "/home/zhzhu/workspace/src/path_planning/lib/matplotlib-cpp")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
SET(Eigen3_DIR "/usr/lib/cmake/eigen3/") # or include /usr/include/eigen3/

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        geometry_msgs
        Eigen3 REQUIRED
        visualization_msgs
)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Segment.msg
        Viewpoint.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES path_planning
          CATKIN_DEPENDS message_runtime geometry_msgs std_msgs roscpp
        #  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/lib
        ${MATPLOTLIBCPP_DIR}
        
)


add_executable(frontier_evaluation src/frontier_evaluation.cpp)
add_executable(visulization src/visulization.cpp)
add_executable(optimal_path_NBV src/optimal_path_NBV.cpp)
add_executable(metrics src/metrics.cpp)


target_link_libraries(frontier_evaluation PUBLIC
   ${catkin_LIBRARIES}
         Eigen3::Eigen
 )

 target_link_libraries(optimal_path_NBV PUBLIC
   ${catkin_LIBRARIES}
         Eigen3::Eigen
 )

target_link_libraries(visulization PUBLIC
        ${catkin_LIBRARIES}
        Eigen3::Eigen    
)

target_link_libraries(metrics PUBLIC
        ${catkin_LIBRARIES}
        Eigen3::Eigen 
        Python3::Python
)