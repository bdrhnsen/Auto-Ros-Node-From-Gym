#!/usr/bin/env python3
from typing import Dict, Any
import yaml
import json
from pathlib import Path


class EnvConfigGenerator:
    """Generator for environment configuration files."""
    
    @staticmethod
    def generate_config(env_specs: Dict[str, Any], output_dir: str,
                       env_name: str) -> Dict[str, str]:
        """Generate configuration files for an environment.
        
        Args:
            env_specs: Environment specifications
            output_dir: Directory to save configuration files
            env_name: Name of the environment
            
        Returns:
            Dictionary mapping config type to file path
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        configs = {}
        
        # Generate YAML config for environment parameters
        env_config = {
            'environment': {
                'id': env_name,
                'max_episode_steps': env_specs.get('max_episode_steps'),
                'render_modes': env_specs.get('render_modes', []),
                'reward_range': env_specs['reward_range'],
                'observation_space': {
                    'type': env_specs['observation_space'].space_type,
                    'shape': env_specs['observation_space'].shape,
                    'bounds': env_specs['observation_space'].bounds
                },
                'action_space': {
                    'type': env_specs['action_space'].space_type,
                    'shape': env_specs['action_space'].shape,
                    'bounds': env_specs['action_space'].bounds
                }
            },
            'ros2': {
                'node_name': f'{env_name}_node',
                'namespace': '~',
                'qos': {
                    'reliability': 'BEST_EFFORT',
                    'durability': 'VOLATILE',
                    'history': 'KEEP_LAST',
                    'depth': 1
                }
            }
        }
        
        yaml_path = output_dir / f'{env_name}_config.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(env_config, f, default_flow_style=False)
        configs['yaml'] = str(yaml_path)
        
        # Generate JSON schema for runtime configuration
        schema = {
            'type': 'object',
            'properties': {
                'environment': {
                    'type': 'object',
                    'properties': {
                        'seed': {'type': 'integer'},
                        'render_mode': {'type': 'string', 'enum': env_specs.get('render_modes', [])},
                        'max_episode_steps': {'type': 'integer', 'minimum': 1}
                    }
                }
            }
        }
        
        schema_path = output_dir / f'{env_name}_schema.json'
        with open(schema_path, 'w') as f:
            json.dump(schema, f, indent=2)
        configs['schema'] = str(schema_path)
        
        return configs 