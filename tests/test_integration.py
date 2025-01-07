#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import gymnasium as gym
import numpy as np
import tempfile
import shutil
import subprocess
import time
from pathlib import Path
import yaml
import json

from ros_gym_automation.generators.env_parser import GymEnvParser
from ros_gym_automation.generators.node_generator import EnvNodeGenerator
from ros_gym_automation.generators.agent_generator import AgentNodeGenerator
from ros_gym_automation.utils.env_validator import EnvironmentValidator


class TestNodeInteraction(unittest.TestCase):
    """Integration tests for ROS node interaction."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = MultiThreadedExecutor()
        
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_id = 'CartPole-v1'
        self.agent_name = 'test_agent'
        
        # Generate nodes
        self._generate_test_nodes()
        
        # Start ROS nodes
        self.env_process = subprocess.Popen(['ros2', 'run', 'ros_gym_automation', 
                                           f'{self.env_id.replace("-", "_")}_node'])
        self.agent_process = subprocess.Popen(['ros2', 'run', 'ros_gym_automation',
                                             f'{self.agent_name}_node'])
        time.sleep(2)  # Wait for nodes to start
        
    def tearDown(self):
        self.env_process.terminate()
        self.agent_process.terminate()
        shutil.rmtree(self.temp_dir)
        
    def _generate_test_nodes(self):
        """Generate environment and agent nodes for testing."""
        env_generator = EnvNodeGenerator()
        agent_generator = AgentNodeGenerator()
        
        # Generate environment node
        env_node_path = env_generator.generate_node(
            self.env_id,
            'ros_gym_automation',
            self.temp_dir
        )
        
        # Get environment specs
        env_specs = GymEnvParser.get_env_specs(self.env_id)
        
        # Generate agent node
        agent_node_path = agent_generator.generate_node(
            self.agent_name,
            'ros_gym_automation',
            env_specs['observation_space'],
            env_specs['action_space'],
            self.temp_dir
        )
        
    def test_reset_service(self):
        """Test environment reset service."""
        node = Node('test_node')
        self.executor.add_node(node)
        
        client = node.create_client('ResetEnv', f'/{self.env_id}/reset')
        
        # Wait for service
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        
        # Call reset service
        request = ResetEnv.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        self.assertTrue(future.result().success)
        self.assertIsNotNone(future.result().initial_observation)
        
    def test_action_observation_cycle(self):
        """Test the action-observation cycle between nodes."""
        node = Node('test_node')
        self.executor.add_node(node)
        
        # Create subscribers and publishers
        observations = []
        rewards = []
        
        def obs_callback(msg):
            observations.append(msg)
            
        def reward_callback(msg):
            rewards.append(msg.data)
            
        node.create_subscription(
            'Float32MultiArray',
            f'/{self.env_id}/observation',
            obs_callback,
            10
        )
        node.create_subscription(
            'Float32',
            f'/{self.env_id}/reward',
            reward_callback,
            10
        )
        
        # Wait for messages
        time.sleep(2)
        
        self.assertGreater(len(observations), 0)
        self.assertGreater(len(rewards), 0)
        
    def test_environment_lifecycle(self):
        """Test environment node lifecycle transitions."""
        node = Node('test_node')
        self.executor.add_node(node)
        
        # Test configure
        configure_client = node.create_client(
            'ConfigureEnv',
            f'/{self.env_id}/configure'
        )
        self.assertTrue(configure_client.wait_for_service(timeout_sec=5.0))
        
        config = {'render_mode': 'rgb_array'}
        request = ConfigureEnv.Request()
        request.config_yaml = json.dumps(config)
        
        future = configure_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        
        self.assertTrue(future.result().success)


if __name__ == '__main__':
    unittest.main() 