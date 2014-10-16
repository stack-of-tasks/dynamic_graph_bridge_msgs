cmake_minimum_required(VERSION 2.8.3)
project(dynamic_graph_bridge_msgs)

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