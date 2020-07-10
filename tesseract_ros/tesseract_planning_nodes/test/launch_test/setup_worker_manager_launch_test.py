import os
import unittest
import copy

from ament_index_python import get_package_share_directory, get_package_prefix
import shutil
import tempfile

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.node import Node

import launch
import launch.actions
import launch_ros.actions


import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import tf2_ros
from std_srvs.srv import Trigger

from tesseract_msgs.action import SolvePlan
from tesseract_msgs.msg import PlannerConfigurator

from sensor_msgs.msg import JointState

import pytest
import time

@pytest.mark.launch_test
# @launch_testing.markers.keep_alive
def generate_test_description():
    manager = launch_ros.actions.Node(
        node_name='manager_node',
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_manager_node',
        output='screen',
    )

    worker1 = launch_ros.actions.Node(
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_worker_node',
        output='screen',
        parameters=[{
            'robot_description': os.path.join(get_package_share_directory('tesseract_support'), 'urdf', 'abb_irb2400.urdf'),
            'robot_description_semantic': os.path.join(get_package_share_directory('tesseract_support'),
                                                       'urdf', 'abb_irb2400.srdf'),
        }]
    )

    worker2 = launch_ros.actions.Node(
        package='tesseract_planning_nodes',
        node_executable='tesseract_planning_nodes_planning_worker_node',
        output='screen',
        parameters=[{
            'robot_description': os.path.join(get_package_share_directory('tesseract_support'), 'urdf', 'abb_irb2400.urdf'),
            'robot_description_semantic': os.path.join(get_package_share_directory('tesseract_support'),
                                                       'urdf', 'abb_irb2400.srdf'),
        }]
    )

    return launch.LaunchDescription([
        manager,
        worker1,
        worker2,
        launch_testing.actions.ReadyToTest(),
    ])

class SolvePlanClient(Node):
    def __init__(self):
        super().__init__('solve_plan_action_client')
        self._action_client = ActionClient(self, SolvePlan, '/solve_plan')
        self._is_done = False

    def is_done(self):
        return self._is_done

    def solve_plan_goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self._is_done = True
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.solve_plan_result_cb)

    def solve_plan_result_cb(self, future):
        result = future.result().result
        print(result.success)
        self._is_done = True

    def request_solve_plan(self):
        print('request solve plan')

        goal_msg = SolvePlan.Goal()
        goal_msg.planner_config.manipulator = 'manipulator'

        configurator = PlannerConfigurator()
        configurator.range = 0.5
        configurator.type = PlannerConfigurator.RRT_CONNECT
        goal_msg.planner_config.configurators = [configurator]

        names = ['joint_1',
                 'joint_2',
                 'joint_3',
                 'joint_4',
                 'joint_5',
                 'joint_6',]

        start_state = JointState()
        start_state.name = names
        start_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        end_state = JointState()
        end_state.name = names
        end_state.position = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]

        goal_msg.planner_config.start_state = start_state
        goal_msg.planner_config.end_state = end_state
        goal_msg.planner_config.collision_safety_margin = 0.01
        goal_msg.planner_config.simplify = True
        goal_msg.planner_config.planning_time = 30.0
        goal_msg.planner_config.collision_continuous = True
        goal_msg.planner_config.collision_check = True
        goal_msg.planner_config.max_solutions = 1
        goal_msg.planner_config.longest_valid_segment_length = 0.01
        goal_msg.planner_config.n_output_states = 100

        # goal_msg.planner_config.start_state
        self._is_done = False
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.solve_plan_goal_response_cb)


class TestSolvePlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        time.sleep(5)
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self._client = SolvePlanClient()
        # pass

    def tearDown(self):
        self._client.destroy_node()

    def testSolvePlanAction(self, proc_output):
        self._client.request_solve_plan()
        time.sleep(2.0)
        self._client.request_solve_plan()
        time.sleep(30)

        while not self._client.is_done():
            rclpy.spin_once(self._client, timeout_sec=0.1)

        assert(True)

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)