#!/usr/bin/env python3
import unittest
import gymnasium as gym
import numpy as np
from pathlib import Path
import tempfile
import shutil
from PIL import Image

from ros_gym_automation.generators.env_parser import GymEnvParser
from ros_gym_automation.generators.node_generator import EnvNodeGenerator
from ros_gym_automation.utils.env_validator import EnvironmentValidator


class TestDiscreteActionSpaces(unittest.TestCase):
    """Test suite for environments with discrete action spaces."""
    
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_ids = ['CartPole-v1', 'FrozenLake-v1', 'Taxi-v3']
        
    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def test_discrete_environments(self):
        """Test node generation for discrete action space environments."""
        generator = EnvNodeGenerator()
        
        for env_id in self.env_ids:
            with self.subTest(env_id=env_id):
                # Validate environment
                valid, msg, specs = EnvironmentValidator.validate_environment(env_id)
                self.assertTrue(valid, f"Environment {env_id} validation failed: {msg}")
                
                # Generate node
                node_path = generator.generate_node(env_id, 'test_package', self.temp_dir)
                self.assertTrue(Path(node_path).exists())
                
                # Verify action space type
                env = gym.make(env_id)
                self.assertTrue(isinstance(env.action_space, gym.spaces.Discrete))
                env.close()


class TestContinuousActionSpaces(unittest.TestCase):
    """Test suite for environments with continuous action spaces."""
    
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_ids = ['Pendulum-v1', 'MountainCarContinuous-v0']
        
    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def test_continuous_environments(self):
        """Test node generation for continuous action space environments."""
        generator = EnvNodeGenerator()
        
        for env_id in self.env_ids:
            with self.subTest(env_id=env_id):
                # Validate environment
                valid, msg, specs = EnvironmentValidator.validate_environment(env_id)
                self.assertTrue(valid, f"Environment {env_id} validation failed: {msg}")
                
                # Generate node
                node_path = generator.generate_node(env_id, 'test_package', self.temp_dir)
                self.assertTrue(Path(node_path).exists())
                
                # Verify action space type
                env = gym.make(env_id)
                self.assertTrue(isinstance(env.action_space, gym.spaces.Box))
                env.close()


class TestImageBasedObservations(unittest.TestCase):
    """Test suite for environments with image-based observations."""
    
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_ids = ['CarRacing-v2']  # Add more image-based environments as needed
        
    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def test_image_environments(self):
        """Test node generation for image-based environments."""
        generator = EnvNodeGenerator()
        
        for env_id in self.env_ids:
            with self.subTest(env_id=env_id):
                # Validate environment
                valid, msg, specs = EnvironmentValidator.validate_environment(env_id)
                self.assertTrue(valid, f"Environment {env_id} validation failed: {msg}")
                
                # Generate node
                node_path = generator.generate_node(env_id, 'test_package', self.temp_dir)
                self.assertTrue(Path(node_path).exists())
                
                # Verify observation space
                env = gym.make(env_id)
                self.assertTrue(len(env.observation_space.shape) == 3)  # Height, width, channels
                env.close()


class TestVectorBasedObservations(unittest.TestCase):
    """Test suite for environments with vector-based observations."""
    
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_ids = ['CartPole-v1', 'Pendulum-v1', 'Acrobot-v1']
        
    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def test_vector_environments(self):
        """Test node generation for vector-based environments."""
        generator = EnvNodeGenerator()
        
        for env_id in self.env_ids:
            with self.subTest(env_id=env_id):
                # Validate environment
                valid, msg, specs = EnvironmentValidator.validate_environment(env_id)
                self.assertTrue(valid, f"Environment {env_id} validation failed: {msg}")
                
                # Generate node
                node_path = generator.generate_node(env_id, 'test_package', self.temp_dir)
                self.assertTrue(Path(node_path).exists())
                
                # Verify observation space
                env = gym.make(env_id)
                self.assertTrue(len(env.observation_space.shape) == 1)  # 1D vector
                env.close()


if __name__ == '__main__':
    unittest.main() 