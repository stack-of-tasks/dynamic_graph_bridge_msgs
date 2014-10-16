cmake_minimum_required(VERSION 2.8.3)
project(dynamic_graph_bridge_msgs)

## LAAS cmake submodule part
set(PROJECT_DESCRIPTION "Dynamic graph bridge msgs ")
set(PROJECT_NAME dynamic_graph_bridge_msgs)
set(PROJECT_URL "")

set(CXX_DISABLE_WERROR False)
set(CUSTOM_HEADER_DIR dynamic_graph_bridge_msgs)
include(cmake/base.cmake)
include(cmake/ros.cmake)
include(cmake/GNUInstallDirs.cmake)


SETUP_PROJECT()
#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES std_msgs )

#Declare build dependencies
find_package(catkin REQUIRED
COMPONENTS
message_generation
${MESSAGE_DEPENDENCIES} )

#Add message files
add_message_files(DIRECTORY msg FILES Matrix.msg Vector.msg)

#Add service files
add_service_files(DIRECTORY srv
FILES
RunCommand.srv
RunPythonFile.srv)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

# Generate catkin/pkg-config import information
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES} )


SETUP_PROJECT_FINALIZE()