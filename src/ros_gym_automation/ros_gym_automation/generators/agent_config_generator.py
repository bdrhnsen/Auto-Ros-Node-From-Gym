#!/usr/bin/env python3
from typing import Dict, Any
import yaml
import json
from pathlib import Path


class AgentConfigGenerator:
    """Generator for agent configuration files."""
    
    @staticmethod
    def generate_config(agent_name: str, observation_space: Dict,
                       action_space: Dict, output_dir: str,
                       extra_params: Dict[str, Any] = None) -> Dict[str, str]:
        """Generate configuration files for an agent.
        
        Args:
            agent_name: Name of the agent
            observation_space: Specification of observation space
            action_space: Specification of action space
            output_dir: Directory to save configuration files
            extra_params: Additional parameters for the agent
            
        Returns:
            Dictionary mapping config type to file path
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        configs = {}
        
        # Generate YAML config for agent parameters
        agent_config = {
            'agent': {
                'name': agent_name,
                'type': extra_params.get('agent_type', 'default'),
                'model_path': extra_params.get('model_path', ''),
                'observation_space': observation_space,
                'action_space': action_space,
                'parameters': extra_params.get('parameters', {}),
            },
            'ros2': {
                'node_name': f'{agent_name}_node',
                'namespace': '~',
                'qos': {
                    'reliability': 'BEST_EFFORT',
                    'durability': 'VOLATILE',
                    'history': 'KEEP_LAST',
                    'depth': 1
                }
            }
        }
        
        yaml_path = output_dir / f'{agent_name}_config.yaml'
        with open(yaml_path, 'w') as f:
            yaml.dump(agent_config, f, default_flow_style=False)
        configs['yaml'] = str(yaml_path)
        
        # Generate JSON schema for runtime configuration
        schema = {
            'type': 'object',
            'properties': {
                'agent': {
                    'type': 'object',
                    'properties': {
                        'model_path': {'type': 'string'},
                        'parameters': {
                            'type': 'object',
                            'additionalProperties': True
                        },
                        'learning_rate': {'type': 'number', 'minimum': 0},
                        'batch_size': {'type': 'integer', 'minimum': 1},
                        'update_frequency': {'type': 'integer', 'minimum': 1}
                    }
                }
            }
        }
        
        schema_path = output_dir / f'{agent_name}_schema.json'
        with open(schema_path, 'w') as f:
            json.dump(schema, f, indent=2)
        configs['schema'] = str(schema_path)
        
        return configs 