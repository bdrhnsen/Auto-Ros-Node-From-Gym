#!/usr/bin/env python3
import unittest
import gymnasium as gym
import numpy as np
from pathlib import Path
import tempfile
import shutil

from ros_gym_automation.generators.env_parser import GymEnvParser, SpaceSpec
from ros_gym_automation.generators.node_generator import EnvNodeGenerator
from ros_gym_automation.generators.agent_generator import AgentNodeGenerator
from ros_gym_automation.utils.env_validator import EnvironmentValidator
from ros_gym_automation.utils.model_loader import ModelLoader


class TestEnvironmentParser(unittest.TestCase):
    """Test suite for environment parser functionality."""
    
    def setUp(self):
        self.discrete_env = gym.make('CartPole-v1')
        self.continuous_env = gym.make('Pendulum-v1')
        
    def tearDown(self):
        self.discrete_env.close()
        self.continuous_env.close()
        
    def test_parse_discrete_space(self):
        """Test parsing of discrete action space."""
        specs = GymEnvParser.get_env_specs(self.discrete_env)
        self.assertEqual(specs['action_space'].space_type, 'Discrete')
        self.assertEqual(specs['action_space'].n, 2)
        
    def test_parse_continuous_space(self):
        """Test parsing of continuous action space."""
        specs = GymEnvParser.get_env_specs(self.continuous_env)
        self.assertEqual(specs['action_space'].space_type, 'Box')
        self.assertEqual(specs['action_space'].shape, (1,))
        
    def test_parse_observation_space(self):
        """Test parsing of observation space."""
        specs = GymEnvParser.get_env_specs(self.discrete_env)
        self.assertEqual(specs['observation_space'].space_type, 'Box')
        self.assertEqual(specs['observation_space'].shape, (4,))


class TestNodeGenerator(unittest.TestCase):
    """Test suite for node generator functionality."""
    
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.env_generator = EnvNodeGenerator()
        self.agent_generator = AgentNodeGenerator()
        
    def tearDown(self):
        shutil.rmtree(self.temp_dir)
        
    def test_generate_env_node(self):
        """Test environment node generation."""
        node_path = self.env_generator.generate_node(
            'CartPole-v1',
            'test_package',
            self.temp_dir
        )
        self.assertTrue(Path(node_path).exists())
        
    def test_generate_agent_node(self):
        """Test agent node generation."""
        env_specs = GymEnvParser.get_env_specs('CartPole-v1')
        node_path = self.agent_generator.generate_node(
            'test_agent',
            'test_package',
            env_specs['observation_space'],
            env_specs['action_space'],
            self.temp_dir
        )
        self.assertTrue(Path(node_path).exists())


class TestEnvironmentValidator(unittest.TestCase):
    """Test suite for environment validator functionality."""
    
    def test_valid_environment(self):
        """Test validation of a valid environment."""
        valid, msg, specs = EnvironmentValidator.validate_environment('CartPole-v1')
        self.assertTrue(valid)
        self.assertIn('observation_space', specs)
        self.assertIn('action_space', specs)
        
    def test_invalid_environment(self):
        """Test validation of an invalid environment ID."""
        valid, msg, specs = EnvironmentValidator.validate_environment('NonexistentEnv-v0')
        self.assertFalse(valid)
        self.assertEqual(specs, {})


if __name__ == '__main__':
    unittest.main() 