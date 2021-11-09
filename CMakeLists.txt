cmake_minimum_required(VERSION 3.5)

set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic_graph_bridge_msgs)
set(PROJECT_DESCRIPTION "Dynamic graph bridge msgs ")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

set(CUSTOM_HEADER_DIR dynamic_graph_bridge_msgs)
if(DEFINED ENV{ROS_VERSION} )
  # ROS-1
  if ($ENV{ROS_VERSION} EQUAL 1)
    set(PROJECT_USE_CMAKE_EXPORT TRUE)
  endif()
endif()

include(cmake/base.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX C)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(DEFINED ENV{ROS_VERSION} )

  # ROS-1
  if ($ENV{ROS_VERSION} EQUAL 1)
    # Declare build dependencies
    find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

    # Add message files
    add_message_files(DIRECTORY msg FILES Matrix.msg Vector.msg)

    # Add service files
    add_service_files(
        DIRECTORY srv
        FILES RunCommand.srv RunPythonCommand.srv RunPythonFile.srv
    )

    # And now generate the messages
    generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

    add_library(${PROJECT_NAME} INTERFACE)

    install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
    install(FILES package.xml DESTINATION share/${PROJECT_NAME})

  endif()

  # IF ROS2
  if ($ENV{ROS_VERSION} EQUAL 2)
    # Dependency to roscpp is also optional
    cmake_policy(SET CMP0057 NEW)
    # Default to C99
    if(NOT CMAKE_C_STANDARD)
      set(CMAKE_C_STANDARD 99)
    endif()

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Matrix.msg"
      "msg/Vector.msg"
      "srv/RunCommand.srv"
      "srv/RunPythonCommand.srv"
      "srv/RunPythonFile.srv"
      )

    #    install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
    install(FILES package.xml DESTINATION share/${PROJECT_NAME})

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()

  endif()
endif()
