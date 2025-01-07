#!/usr/bin/env python3
from typing import Dict, Any, List
import os
from pathlib import Path
from .env_parser import SpaceSpec


class InterfaceGenerator:
    """Generator for ROS2 interface definitions."""
    
    def __init__(self, output_dir: str):
        """Initialize the interface generator.
        
        Args:
            output_dir: Directory where to save generated interfaces
        """
        self.output_dir = Path(output_dir)
        self.msg_dir = self.output_dir / 'msg'
        self.srv_dir = self.output_dir / 'srv'
        
        # Create directories
        self.msg_dir.mkdir(parents=True, exist_ok=True)
        self.srv_dir.mkdir(parents=True, exist_ok=True)
        
    def generate_interfaces(self, env_specs: Dict[str, Any]) -> Dict[str, List[str]]:
        """Generate all required interfaces for an environment.
        
        Args:
            env_specs: Environment specifications
            
        Returns:
            Dictionary mapping interface type to list of generated files
        """
        generated_files = {'msg': [], 'srv': []}
        
        # Generate message definitions
        generated_files['msg'].extend(self._generate_observation_msg(env_specs['observation_space']))
        generated_files['msg'].extend(self._generate_action_msg(env_specs['action_space']))
        generated_files['msg'].extend(self._generate_status_msg())
        
        # Generate service definitions
        generated_files['srv'].extend(self._generate_reset_srv())
        generated_files['srv'].extend(self._generate_configure_srv())
        
        return generated_files
        
    def _generate_observation_msg(self, space_spec: SpaceSpec) -> List[str]:
        """Generate observation message definition."""
        msg_path = self.msg_dir / 'Observation.msg'
        with open(msg_path, 'w') as f:
            if space_spec.space_type == 'Box':
                f.write('# Observation message for Box space\n')
                f.write('float32[] data\n')
                f.write(f'int32[] shape {list(space_spec.shape)}  # Fixed shape\n')
            elif space_spec.space_type == 'Discrete':
                f.write('# Observation message for Discrete space\n')
                f.write('int32 data\n')
            elif space_spec.space_type == 'MultiBinary':
                f.write('# Observation message for MultiBinary space\n')
                f.write('bool[] data\n')
        return [str(msg_path)]
        
    def _generate_action_msg(self, space_spec: SpaceSpec) -> List[str]:
        """Generate action message definition."""
        msg_path = self.msg_dir / 'Action.msg'
        with open(msg_path, 'w') as f:
            if space_spec.space_type == 'Box':
                f.write('# Action message for Box space\n')
                f.write('float32[] data\n')
            elif space_spec.space_type == 'Discrete':
                f.write('# Action message for Discrete space\n')
                f.write('int32 data\n')
        return [str(msg_path)]
        
    def _generate_status_msg(self) -> List[str]:
        """Generate environment status message definition."""
        msg_path = self.msg_dir / 'EnvStatus.msg'
        with open(msg_path, 'w') as f:
            f.write('# Environment status message\n')
            f.write('bool ready\n')
            f.write('string state\n')
            f.write('string[] available_actions\n')
            f.write('string[] active_modes\n')
        return [str(msg_path)]
        
    def _generate_reset_srv(self) -> List[str]:
        """Generate reset service definition."""
        srv_path = self.srv_dir / 'ResetEnv.srv'
        with open(srv_path, 'w') as f:
            f.write('# Reset environment service\n')
            f.write('bool seed_enabled\n')
            f.write('int32 seed\n')
            f.write('---\n')
            f.write('bool success\n')
            f.write('string message\n')
            f.write('float32[] initial_observation\n')
        return [str(srv_path)]
        
    def _generate_configure_srv(self) -> List[str]:
        """Generate configure service definition."""
        srv_path = self.srv_dir / 'ConfigureEnv.srv'
        with open(srv_path, 'w') as f:
            f.write('# Configure environment service\n')
            f.write('string config_yaml\n')
            f.write('---\n')
            f.write('bool success\n')
            f.write('string message\n')
        return [str(srv_path)] 