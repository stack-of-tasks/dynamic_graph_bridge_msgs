"""
license BSD 3-clause
Copyright (c) 2020, CNRS

Unit-tests for the python API of the DynamicGraphManager
"""

from pathlib import Path
import sys
import unittest
import pytest
import rclpy
from rclpy.node import Node

import launch
import launch_ros
import launch_ros.actions
try:
    import launch_testing.actions
    is_launch_testing = True
except ImportError:
    is_launch_testing = False


from dynamic_graph_bridge_msgs.srv import (
    RunPythonCommand,
    RunCommand,
    RunPythonFile,
)


@pytest.mark.rostest
def generate_test_description():
    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through
    path_to_test = Path(__file__).resolve().parent

    server_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[str(path_to_test / 'unit_test_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'}
    )

    if is_launch_testing:
        return (
            launch.LaunchDescription([
                server_node,
                # Start tests right away - no need to wait for anything
                launch_testing.actions.ReadyToTest(),
            ]),
            {
                'python_server_node': server_node
            },
        )
    else:
        return (
            launch.LaunchDescription([
                server_node,
            ]),
            {
                'python_server_node': server_node
            },
        )



class RunPythonCommandClient(Node):

    def __init__(self):
        super().__init__('run_python_command_client_node')
        self.cli = self.create_client(RunPythonCommand, 'run_python_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunPythonCommand.Request()

    def send_request(self):
        self.req.input = "1+1"
        self.future = self.cli.call_async(self.req)


class RunCommandClient(Node):

    def __init__(self):
        super().__init__('run_command_client_node')
        self.cli = self.create_client(RunCommand, 'run_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunCommand.Request()

    def send_request(self):
        self.req.input = "1+1"
        self.future = self.cli.call_async(self.req)


class RunPythonFileClient(Node):

    def __init__(self):
        super().__init__('run_python_file_client_node')
        self.cli = self.create_client(RunPythonFile, 'run_python_file')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunPythonFile.Request()

    def send_good_request(self):
        self.req.input = str(Path(__file__).resolve())
        self.future = self.cli.call_async(self.req)

    def send_bad_request(self):
        self.req.input = "hthre21@#$%@)#_#%*+($^&$i;gnvj;bae"
        self.future = self.cli.call_async(self.req)


class TestPythonServices(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_run_python_command(self):
        client = RunPythonCommandClient()
        client.send_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info('Result acquired')
                break

        self.assertEqual(client.req.input, "1+1")
        self.assertEqual(response.result, "1+1_result_python_cmd")
        self.assertEqual(response.standardoutput, "standardoutput")
        self.assertEqual(response.standarderror, "standarderror")
        
        client.destroy_node()

    def test_run_command(self):
        client = RunCommandClient()
        client.send_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info('Result acquired')
                break

        self.assertEqual(client.req.input, "1+1")
        self.assertEqual(response.result, "1+1_result_cmd")
        self.assertEqual(response.standardoutput, "standardoutput")
        self.assertEqual(response.standarderror, "standarderror")

        client.destroy_node()

    def test_run_python_file_good(self):
        client = RunPythonFileClient()
        client.send_good_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info('Result acquired')
                break

        self.assertEqual(client.req.input, str(Path(__file__)))
        self.assertEqual(response.result, "True")

        client.destroy_node()

    def test_run_python_file_bad(self):
        client = RunPythonFileClient()
        client.send_bad_request()

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.future.done():
                try:
                    response = client.future.result()
                except Exception as e:
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    client.get_logger().info('Result acquired')
                break

        self.assertEqual(client.req.input, "hthre21@#$%@)#_#%*+($^&$i;gnvj;bae")
        self.assertEqual(response.result, "False")

        client.destroy_node()
