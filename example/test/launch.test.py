# Copyright 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time
import unittest
from launch_ros.substitutions import FindPackageShare

import pytest
import rclpy
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc


# This function specifies the processes to be run for our test
@pytest.mark.rostest
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    return LaunchDescription(
        [
            Node(
                package='generate_parameter_library_example',
                executable='test_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare('generate_parameter_library_example'),
                            'config',
                            'implementation.yaml',
                        ]
                    )
                ],
            ),
            KeepAliveProc(),
            ReadyToTest(),
        ],
    )


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_check_node(self, proc_output):
        node_name = 'admittance_controller'
        start = time.time()
        found = False
        while time.time() - start < 5.0 and not found:
            found = node_name in self.node.get_node_names()
            time.sleep(0.1)
        assert found, f"{node_name} not found!"


@launch_testing.post_shutdown_test()
class TestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_pass(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
