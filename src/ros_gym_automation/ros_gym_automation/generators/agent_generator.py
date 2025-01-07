#!/usr/bin/env python3
from typing import Dict, Any
import os
from jinja2 import Environment, FileSystemLoader
from pathlib import Path
from .env_parser import SpaceSpec
from .msg_generator import ROSMsgGenerator


class AgentNodeGenerator:
    """Generator for ROS2 agent nodes."""
    
    def __init__(self, template_dir: str = None):
        """Initialize the agent node generator."""
        if template_dir is None:
            template_dir = str(Path(__file__).parent.parent / 'templates')
            
        self.env = Environment(
            loader=FileSystemLoader(template_dir),
            trim_blocks=True,
            lstrip_blocks=True
        )
        self.msg_generator = ROSMsgGenerator()
        
    def generate_node(self, agent_name: str, package_name: str,
                     observation_space: SpaceSpec,
                     action_space: SpaceSpec,
                     output_dir: str,
                     agent_config: Dict[str, Any] = None) -> str:
        """Generate a ROS2 node for an RL agent.
        
        Args:
            agent_name: Name of the agent
            package_name: Name of the ROS2 package
            observation_space: Specification of the observation space
            action_space: Specification of the action space
            output_dir: Directory where to save the generated node
            agent_config: Optional agent configuration
            
        Returns:
            Path to the generated node file
        """
        # Generate conversion methods
        obs_conversion = self._generate_observation_processing(observation_space)
        action_conversion = self._generate_action_publishing(action_space)
        
        # Prepare template variables
        template_vars = {
            'agent_name': agent_name,
            'package_name': package_name,
            'observation_type': self._get_python_type(observation_space),
            'action_type': self._get_python_type(action_space),
            'observation_msg_type': self.msg_generator.space_to_msg_type(observation_space),
            'action_msg_type': self.msg_generator.space_to_msg_type(action_space),
            'observation_conversion_code': obs_conversion,
            'action_conversion_code': action_conversion,
            'config': agent_config or {}
        }
        
        # Generate node code
        template = self.env.get_template('agent_node_template.py.jinja2')
        node_code = template.render(**template_vars)
        
        # Save to file
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"{agent_name.lower()}_node.py")
        with open(output_path, 'w') as f:
            f.write(node_code)
            
        # Make file executable
        os.chmod(output_path, 0o755)
        
        return output_path
        
    def _generate_observation_processing(self, space_spec: SpaceSpec) -> str:
        """Generate code for processing observations."""
        if space_spec.space_type == 'Box':
            return """
            # Convert observation array to numpy array with correct shape
            observation_data = np.array(msg.data)
            if len(self.observation_space.shape) > 1:
                observation_data = observation_data.reshape(self.observation_space.shape)
            return observation_data
            """
        elif space_spec.space_type == 'Discrete':
            return """
            return msg.data
            """
        elif space_spec.space_type == 'MultiBinary':
            return """
            return np.array(msg.data, dtype=bool)
            """
        else:
            raise ValueError(f"Unsupported observation space type: {space_spec.space_type}")
            
    def _generate_action_publishing(self, space_spec: SpaceSpec) -> str:
        """Generate code for publishing actions."""
        if space_spec.space_type == 'Box':
            return """
            msg = Action()
            msg.data = action.flatten().tolist()
            return msg
            """
        elif space_spec.space_type == 'Discrete':
            return """
            msg = Action()
            msg.data = int(action)
            return msg
            """
        else:
            raise ValueError(f"Unsupported action space type: {space_spec.space_type}")
            
    def _get_python_type(self, space_spec: SpaceSpec) -> str:
        """Get the Python type annotation for a space specification."""
        if space_spec.space_type == 'Box':
            return 'np.ndarray'
        elif space_spec.space_type == 'Discrete':
            return 'int'
        elif space_spec.space_type == 'MultiBinary':
            return 'np.ndarray'
        else:
            return 'Any' 