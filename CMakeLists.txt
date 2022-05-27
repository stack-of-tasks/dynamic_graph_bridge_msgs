cmake_minimum_required(VERSION 3.5)

set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic_graph_bridge_msgs)
set(PROJECT_DESCRIPTION "Dynamic graph bridge msgs ")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  set(ROS_VERSION 1)
endif()

if(ROS_VERSION EQUAL 1) # if ROS1
  # Here we use the jrl-cmakemodules to package things properly.
  set(CUSTOM_HEADER_DIR dynamic_graph_bridge_msgs)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
  include(cmake/base.cmake)

  COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX C)
  project(${PROJECT_NAME} ${PROJECT_ARGS})

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
  target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)
  install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

else() # if ROS2
  # Here ament is doing things ok.
  project(${PROJECT_NAME})

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Matrix.msg"
    "msg/Vector.msg"
    "srv/RunCommand.srv"
    "srv/RunPythonCommand.srv"
    "srv/RunPythonFile.srv"
  )
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  if(ROS_VERSION EQUAL 1) # if ROS1
    find_package(rostest REQUIRED)
    add_rostest(tests/ros1/unit_test_ros.test)
  else() # if ROS-2
    include(CTest)
    include(cmake/python.cmake)
    findpython()
    add_test(
      NAME test_${PROJECT_NAME}_python
      COMMAND ${PYTHON_EXECUTABLE} -m pytest -v
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tests/ros2)
    set_tests_properties(
      test_${PROJECT_NAME}_python
      PROPERTIES
        ENVIRONMENT
        "PYTHONPATH=${CMAKE_BINARY_DIR}/rosidl_generator_py/:${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}"
    )
  endif()
endif() # BUILD_TESTING
