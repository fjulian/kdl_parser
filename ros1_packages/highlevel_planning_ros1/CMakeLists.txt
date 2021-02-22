cmake_minimum_required(VERSION 3.0.2)
project(highlevel_planning_ros1)

add_compile_options(-std=c++11)

find_package(catkin QUIET COMPONENTS message_generation std_msgs)

if (catkin_FOUND)
    catkin_python_setup()

    add_message_files(FILES HlpGuiCommand.msg)
    add_service_files(FILES Snapshot.srv)

    generate_messages(DEPENDENCIES std_msgs)

    catkin_package(CATKIN_DEPENDS message_runtime)
    include_directories()
endif()
