# Dynamic Graph Bridge MSGS

[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge_msgs/badges/master/pipeline.svg)](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge_msgs/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/dynamic_graph_bridge_msgs/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/stack-of-tasks/dynamic_graph_bridge_msgs/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/stack-of-tasks/dynamic_graph_bridge_msgs/master.svg)](https://results.pre-commit.ci/latest/github/stack-of-tasks/dynamic_graph_bridge_msgs)

This packages provides the ROS1/2 messages and services for the user interface
of the
[SoT](https://stack-of-tasks.github.io/sot-doc/doxygen/HEAD/page_overview.html)
framework.

The current messages are:
- `msg/Vector.msg` representing a dynamicaly resizable vector of double.
- `msg/Matrix.msg` representing a dynamicaly resizable matrix of double in a row major
format.

The services correspond to the user interface of the SoT framwork. It allows the
user to interact with the C++ embeded python interpreter:
- `srv/RunCommand.srv` is a service allowing the user to execute a python
command in the C++ embeded python interpreter.
- `srv/RunPythonCommand.srv` is symbolic link to the `srv/RunCommand.srv` file.
- `srv/RunPythonFile.srv`is service allowing the user to parse a full script
inside the the C++ embeded python interpreter.

# ROS1/2 build.

The package is building the message for ROS1 or ROS2 dependending on the
currently active environment.
We use the `ROS_VERSION` environment variable in order to handle the different
build.

The ROS1 build is based on the `catkin` cmake module, and the ROS2 is based on
`ament` cmake module.
