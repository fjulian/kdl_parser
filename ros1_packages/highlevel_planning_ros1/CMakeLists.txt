cmake_minimum_required(VERSION 3.0.2)
project(highlevel_planning_ros1)

find_package(catkin QUIET)

if (catkin_FOUND)
    catkin_python_setup()
    catkin_package()
    include_directories()
endif()
