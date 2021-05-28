cmake_minimum_required(VERSION 3.1)

set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic_graph_bridge_msgs)
set(PROJECT_DESCRIPTION "Dynamic graph bridge msgs ")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

set(CUSTOM_HEADER_DIR dynamic_graph_bridge_msgs)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(DEFINED ENV{ROS_VERSION} )

  # ROS-1
  if ($ENV{ROS_VERSION} EQUAL 1)
    # Declare build dependencies
    find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

    # Add message files
    add_message_files(DIRECTORY msg FILES Matrix.msg Vector.msg)

    # Add service files
    add_service_files(DIRECTORY srv FILES RunCommand.srv RunPythonFile.srv)

    # And now generate the messages
    generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})
  endif()

  # IF ROS2
  if ($ENV{ROS_VERSION} EQUAL 2)
    # Dependency to roscpp is also optional
    add_project_dependency(ros)
    find_package(ament_cmake REQUIRED)
  endif()
endif()

add_library(${PROJECT_NAME} INTERFACE)
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
