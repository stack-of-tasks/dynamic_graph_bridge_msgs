#!/usr/bin/env python
"""python_service_server

license BSD 3-clause
Copyright (c) 2020, CNRS

small server for running unit-test on the generated API of ROS2 services.
"""

import os
import rospy
from dynamic_graph_bridge_msgs.srv import (
    RunPythonCommand,
    RunCommand,
    RunPythonFile,
    RunPythonCommandResponse,
    RunCommandResponse,
    RunPythonFileResponse,
)


class ServerNode(object):

    def __init__(self):
        self._run_python_command_srv = rospy.Service(
            'run_python_command', RunPythonCommand, self._run_python_command)

        self._run_command_srv = rospy.Service(
            'run_command', RunCommand, self._run_command)
        
        self._run_python_file_srv = rospy.Service(
            'run_python_file', RunPythonFile, self._run_python_file)
    
    def _run_python_command(self, request):
        response = RunPythonCommandResponse()
        response.result = request.input + "_result_python_cmd"
        response.standardoutput = "standardoutput"
        response.standarderror = "standarderror"
        return response

    def _run_command(self, request):
        response = RunCommandResponse()
        response.result = request.input + "_result_cmd"
        response.standardoutput = "standardoutput"
        response.standarderror = "standarderror"
        return response

    def _run_python_file(self, request):
        response = RunPythonFileResponse()
        response.result = os.path.exists(request.input)
        return response

def main(args=None):
    rospy.init_node('unit_test_node')
    server = ServerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
