#!/usr/bin/env python3
from typing import Dict, Any, Tuple
import gymnasium as gym
from ..generators.env_parser import GymEnvParser, SpaceSpec


class EnvironmentValidator:
    """Utility for validating Gym environments."""
    
    @staticmethod
    def validate_environment(env_id: str) -> Tuple[bool, str, Dict[str, Any]]:
        """Validate a Gym environment and extract its specifications.
        
        Args:
            env_id: Gymnasium environment ID
            
        Returns:
            Tuple containing:
            - Boolean indicating if validation passed
            - String message describing validation result
            - Dictionary containing validated environment specifications
        """
        try:
            # Try to create the environment
            env = gym.make(env_id)
            
            # Extract specifications
            env_specs = GymEnvParser.get_env_specs(env_id)
            
            # Validate basic requirements
            if not isinstance(env.observation_space, (gym.spaces.Box, 
                                                    gym.spaces.Discrete,
                                                    gym.spaces.MultiBinary)):
                return False, "Unsupported observation space type", {}
                
            if not isinstance(env.action_space, (gym.spaces.Box,
                                               gym.spaces.Discrete)):
                return False, "Unsupported action space type", {}
                
            # Validate reward structure
            if not hasattr(env, 'reward_range'):
                return False, "Environment missing reward_range attribute", {}
                
            # Test environment functionality
            obs, _ = env.reset()
            if obs is None:
                return False, "Environment reset failed to return observation", {}
                
            # Try a sample action
            action = env.action_space.sample()
            try:
                obs, reward, terminated, truncated, info = env.step(action)
            except Exception as e:
                return False, f"Environment step failed: {e}", {}
                
            env.close()
            
            return True, "Environment validation successful", env_specs
            
        except Exception as e:
            return False, f"Environment validation failed: {e}", {} 