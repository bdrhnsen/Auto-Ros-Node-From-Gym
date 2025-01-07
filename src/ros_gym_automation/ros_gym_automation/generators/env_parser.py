#!/usr/bin/env python3
from typing import Dict, Any, Tuple
import numpy as np
from gymnasium.spaces import Box, Discrete, MultiBinary, MultiDiscrete, Dict as GymDict
from dataclasses import dataclass


@dataclass
class SpaceSpec:
    """Specification of a Gym space."""
    space_type: str
    shape: Tuple
    dtype: np.dtype
    bounds: Tuple[float, float] = None  # For Box spaces
    n: int = None  # For Discrete spaces
    nvec: np.ndarray = None  # For MultiDiscrete spaces


class GymEnvParser:
    """Parser for extracting specifications from Gym environments."""
    
    @staticmethod
    def parse_space(space: Any) -> SpaceSpec:
        """Parse a Gym space into a SpaceSpec."""
        if isinstance(space, Box):
            return SpaceSpec(
                space_type='Box',
                shape=space.shape,
                dtype=space.dtype,
                bounds=(
                    float(np.min(space.low)),
                    float(np.max(space.high))
                )
            )
        elif isinstance(space, Discrete):
            return SpaceSpec(
                space_type='Discrete',
                shape=(1,),
                dtype=np.int32,
                n=space.n
            )
        elif isinstance(space, MultiBinary):
            return SpaceSpec(
                space_type='MultiBinary',
                shape=space.shape,
                dtype=np.bool_
            )
        elif isinstance(space, MultiDiscrete):
            return SpaceSpec(
                space_type='MultiDiscrete',
                shape=space.shape,
                dtype=np.int32,
                nvec=space.nvec
            )
        elif isinstance(space, GymDict):
            raise NotImplementedError("Dict spaces not yet supported")
        else:
            raise ValueError(f"Unsupported space type: {type(space)}")

    @staticmethod
    def get_env_specs(env: Any) -> Dict[str, Any]:
        """Extract specifications from a Gym environment.
        
        Args:
            env: A Gymnasium environment instance
            
        Returns:
            Dictionary containing environment specifications
        """
        specs = {
            'observation_space': GymEnvParser.parse_space(env.observation_space),
            'action_space': GymEnvParser.parse_space(env.action_space),
            'reward_range': env.reward_range,
            'metadata': env.metadata,
        }
        
        # Try to extract additional information that might be available
        try:
            specs['render_modes'] = env.metadata.get('render_modes', [])
        except AttributeError:
            specs['render_modes'] = []
            
        try:
            specs['max_episode_steps'] = env.spec.max_episode_steps
        except AttributeError:
            specs['max_episode_steps'] = None
            
        return specs


class EnvSpecValidator:
    """Validator for environment specifications."""
    
    @staticmethod
    def validate_specs(specs: Dict[str, Any]) -> Tuple[bool, str]:
        """Validate environment specifications.
        
        Args:
            specs: Dictionary of environment specifications
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        required_keys = ['observation_space', 'action_space', 'reward_range']
        
        # Check required keys
        for key in required_keys:
            if key not in specs:
                return False, f"Missing required specification: {key}"
        
        # Validate observation space
        obs_space = specs['observation_space']
        if not isinstance(obs_space, SpaceSpec):
            return False, "Invalid observation space specification"
            
        # Validate action space
        action_space = specs['action_space']
        if not isinstance(action_space, SpaceSpec):
            return False, "Invalid action space specification"
            
        # Validate reward range
        reward_range = specs['reward_range']
        if not isinstance(reward_range, tuple) or len(reward_range) != 2:
            return False, "Invalid reward range specification"
            
        return True, "Specifications are valid" 