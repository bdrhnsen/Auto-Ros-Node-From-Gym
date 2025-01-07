#!/usr/bin/env python3
from typing import Any, Dict, Optional, Tuple, Union
import numpy as np
import gymnasium as gym
from rclpy.node import Node


class GymEnvWrapper:
    """Base wrapper class for Gym environments that handles ROS2 conversions."""
    
    def __init__(self, env_id: str, env_config: Optional[Dict] = None):
        """Initialize the environment wrapper.
        
        Args:
            env_id: The Gymnasium environment ID
            env_config: Optional configuration dictionary for the environment
        """
        self.env_id = env_id
        self.env_config = env_config or {}
        self.env = gym.make(env_id, **self.env_config)
        
        # Cache space information
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        
    def reset(self) -> Tuple[Any, Dict]:
        """Reset the environment."""
        return self.env.reset()
        
    def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict]:
        """Take a step in the environment."""
        return self.env.step(action)
        
    def close(self) -> None:
        """Close the environment."""
        self.env.close()


class ROS2GymNode(Node):
    """Base ROS2 node class for Gym environment interaction."""
    
    def __init__(self, node_name: str, env_wrapper: GymEnvWrapper):
        """Initialize the ROS2 node.
        
        Args:
            node_name: Name of the ROS2 node
            env_wrapper: Instance of GymEnvWrapper
        """
        super().__init__(node_name)
        self.env_wrapper = env_wrapper
        self.setup_interfaces()
        
    def setup_interfaces(self) -> None:
        """Setup ROS2 publishers, subscribers, and services.
        
        This method should be implemented by derived classes.
        """
        raise NotImplementedError
        
    def convert_to_ros_msg(self, data: Any, msg_type: Any) -> Any:
        """Convert Gym data to ROS message.
        
        This method should be implemented by derived classes.
        """
        raise NotImplementedError
        
    def convert_from_ros_msg(self, msg: Any) -> Any:
        """Convert ROS message to Gym data.
        
        This method should be implemented by derived classes.
        """
        raise NotImplementedError 