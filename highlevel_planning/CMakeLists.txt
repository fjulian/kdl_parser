cmake_minimum_required(VERSION 3.0.2)
project(highlevel_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(FILES HlpGuiCommand.msg)

## Generate services in the 'srv' folder
add_service_files(FILES Snapshot.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chimera_app_highlevel_planning
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_custom_command(OUTPUT box_panda_hand_pb.urdf
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/data/models/parse_xacros.bash
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/models)
add_custom_target(ParseXacros ALL DEPENDS box_panda_hand_pb.urdf)