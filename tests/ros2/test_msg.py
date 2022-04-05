#!/usr/bin/env python
"""
license BSD 3-clause
Copyright (c) 2020, CNRS

Unit-tests for the python API of the DynamicGraphManager
"""

import unittest
import rclpy
import numpy as np

from dynamic_graph_bridge_msgs.msg import Vector, Matrix


class TestMessages(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        self.random_vector = np.random.rand(int(np.random.rand() * 10.0 + 1))
        self.random_matrix = np.random.rand(
            int(np.random.rand() * 10.0 + 2), int(np.random.rand() * 10.0 + 2)
        )

    def tearDown(self):
        pass

    def test_create_vector(self):
        v = Vector()
        v.data = self.random_vector.tolist()

        np.testing.assert_array_equal(v.data, self.random_vector)

    def test_create_matrix(self):
        m = Matrix()
        flat_m = self.random_matrix.reshape(self.random_matrix.size).tolist()
        width = self.random_matrix.shape[1]
        print(flat_m)
        m.data = flat_m
        m.width = width

        np.testing.assert_almost_equal(m.data, flat_m)
        self.assertEqual(m.width, width)
