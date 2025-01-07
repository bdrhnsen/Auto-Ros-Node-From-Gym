#!/usr/bin/env python3
from typing import Dict, Any
import os
from jinja2 import Environment, FileSystemLoader
from pathlib import Path
from .env_parser import GymEnvParser, SpaceSpec
from .msg_generator import ROSMsgGenerator


class EnvNodeGenerator:
    """Generator for ROS2 environment nodes."""
    
    def __init__(self, template_dir: str = None):
        """Initialize the node generator.
        
        Args:
            template_dir: Directory containing templates. If None, uses default templates.
        """
        if template_dir is None:
            template_dir = str(Path(__file__).parent.parent / 'templates')
            
        self.env = Environment(
            loader=FileSystemLoader(template_dir),
            trim_blocks=True,
            lstrip_blocks=True
        )
        self.msg_generator = ROSMsgGenerator()
        
    def generate_node(self, env_id: str, package_name: str, output_dir: str,
                     env_config: Dict[str, Any] = None) -> str:
        """Generate a ROS2 node for a Gym environment.
        
        Args:
            env_id: Gymnasium environment ID
            package_name: Name of the ROS2 package
            output_dir: Directory where to save the generated node
            env_config: Optional environment configuration
            
        Returns:
            Path to the generated node file
        """
        # Create temporary environment to get specs
        import gymnasium as gym
        temp_env = gym.make(env_id, **env_config if env_config else {})
        env_specs = GymEnvParser.get_env_specs(temp_env)
        temp_env.close()
        
        # Generate conversion methods based on space specs
        obs_conversion = self._generate_conversion_methods(
            env_specs['observation_space'], 'observation')
        action_conversion = self._generate_conversion_methods(
            env_specs['action_space'], 'action')
            
        # Prepare template variables
        template_vars = {
            'env_id': env_id,
            'env_name': env_id.replace('-', '_'),
            'package_name': package_name,
            'observation_msg_type': self.msg_generator.space_to_msg_type(
                env_specs['observation_space']),
            'action_msg_type': self.msg_generator.space_to_msg_type(
                env_specs['action_space']),
            'convert_to_ros_msg': obs_conversion['to_ros'],
            'convert_from_ros_msg': action_conversion['from_ros'],
            'max_episode_steps': env_specs['max_episode_steps'],
            'render_modes': env_specs['render_modes']
        }
        
        # Generate node code
        template = self.env.get_template('env_node_template.py.jinja2')
        node_code = template.render(**template_vars)
        
        # Save to file
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"{env_id.lower()}_node.py")
        with open(output_path, 'w') as f:
            f.write(node_code)
            
        # Make file executable
        os.chmod(output_path, 0o755)
        
        return output_path
        
    def _generate_conversion_methods(self, space_spec: SpaceSpec,
                                   data_type: str) -> Dict[str, str]:
        """Generate conversion methods for a space specification.
        
        Args:
            space_spec: The space specification
            data_type: Either 'observation' or 'action'
            
        Returns:
            Dictionary with 'to_ros' and 'from_ros' conversion code
        """
        return {
            'to_ros': self.msg_generator.generate_conversion_code(
                space_spec, 'to_ros'),
            'from_ros': self.msg_generator.generate_conversion_code(
                space_spec, 'from_ros')
        } 