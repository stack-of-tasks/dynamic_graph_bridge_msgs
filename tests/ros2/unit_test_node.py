#!/usr/bin/env python
"""python_service_server

license BSD 3-clause
Copyright (c) 2020, CNRS

small server for running unit-test on the generated API of ROS2 services.
"""

from pathlib import Path
import rclpy
from rclpy.node import Node
from dynamic_graph_bridge_msgs.srv import RunPythonCommand, RunCommand, RunPythonFile


class ServerNode(Node):
    def __init__(self):
        super().__init__("python_node")
        self._run_python_command_srv = self.create_service(
            RunPythonCommand, "run_python_command", self._run_python_command
        )

        self._run_command_srv = self.create_service(
            RunCommand, "run_command", self._run_command
        )

        self._run_python_file_srv = self.create_service(
            RunPythonFile, "run_python_file", self._run_python_file
        )

    def _run_python_command(self, request, response):
        response.result = request.input + "_result_python_cmd"
        response.standardoutput = "standardoutput"
        response.standarderror = "standarderror"
        return response

    def _run_command(self, request, response):
        response.result = request.input + "_result_cmd"
        response.standardoutput = "standardoutput"
        response.standarderror = "standarderror"
        return response

    def _run_python_file(self, request, response):
        response.result = str(Path(request.input).exists())
        return response

    def cleanup(self):
        self.destroy_service(self._run_python_command_srv)
        self.destroy_service(self._run_python_file_srv)


def main(args=None):
    rclpy.init(args=args)

    server = ServerNode()

    while rclpy.ok():
        rclpy.spin_once(server)

    server.cleanup()
    server.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
