#!/usr/bin/env python
"""
license BSD 3-clause
Copyright (c) 2020, CNRS

Unit-tests for the python API of the DynamicGraphManager
"""

import os
import unittest
import rostest
import rospy

from dynamic_graph_bridge_msgs.srv import (
    RunPythonCommand,
    RunCommand,
    RunPythonFile,
    RunPythonCommandRequest,
    RunCommandRequest,
    RunPythonFileRequest,
)


class TestServices(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_run_python_command(self):
        rospy.wait_for_service('run_python_command')
        try:
            client = rospy.ServiceProxy('run_python_command', RunPythonCommand)
            request = RunPythonCommandRequest()
            request.input = "1+1"
            response = client(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.assertTrue(False)
        
        self.assertEqual(response.result, "1+1_result_python_cmd")
        self.assertEqual(response.standardoutput, "standardoutput")
        self.assertEqual(response.standarderror, "standarderror")

    def test_run_command(self):
        rospy.wait_for_service('run_command')
        try:
            client = rospy.ServiceProxy('run_command', RunCommand)
            request = RunCommandRequest()
            request.input = "1+1"
            response = client(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.assertTrue(False)
        
        self.assertEqual(response.result, "1+1_result_cmd")
        self.assertEqual(response.standardoutput, "standardoutput")
        self.assertEqual(response.standarderror, "standarderror")

    def test_run_python_file_good(self):
        rospy.wait_for_service('run_python_file')
        try:
            client = rospy.ServiceProxy('run_python_file', RunPythonFile)
            request = RunPythonFileRequest()
            request.input = os.path.abspath(__file__)
            response = client(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.assertTrue(False)

        self.assertEqual(response.result, "True")
    
    def test_run_python_file_bad(self):
        rospy.wait_for_service('run_python_file')
        try:
            client = rospy.ServiceProxy('run_python_file', RunPythonFile)
            request = RunPythonFileRequest()
            request.input = "hthre21@#$%@)#_#%*+($^&$i;gnvj;bae"
            response = client(request)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            self.assertTrue(False)

        self.assertEqual(response.result, "False")


if __name__ == '__main__':
    rostest.rosrun('dynamic_graph_bridge_msgs', 'test_srv', TestServices)
