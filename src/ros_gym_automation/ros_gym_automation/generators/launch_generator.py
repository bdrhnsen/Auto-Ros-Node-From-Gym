#!/usr/bin/env python3
from typing import Dict, Any, List
from pathlib import Path


class LaunchGenerator:
    """Generator for ROS2 launch files."""
    
    @staticmethod
    def generate_launch_file(package_name: str, env_name: str, agent_name: str,
                           env_config_path: str, agent_config_path: str,
                           output_dir: str) -> str:
        """Generate a ROS2 launch file for environment and agent nodes.
        
        Args:
            package_name: Name of the ROS2 package
            env_name: Name of the environment node
            agent_name: Name of the agent node
            env_config_path: Path to environment configuration file
            agent_config_path: Path to agent configuration file
            output_dir: Directory to save the launch file
            
        Returns:
            Path to the generated launch file
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        launch_content = f"""from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get the package share directory
    pkg_dir = get_package_share_directory('{package_name}')
    
    # Load configurations
    env_config = os.path.join(pkg_dir, 'config', '{env_config_path}')
    agent_config = os.path.join(pkg_dir, 'config', '{agent_config_path}')
    
    # Create node actions
    env_node = Node(
        package='{package_name}',
        executable='{env_name}_node',
        name='{env_name}_node',
        parameters=[env_config],
        output='screen'
    )
    
    agent_node = Node(
        package='{package_name}',
        executable='{agent_name}_node',
        name='{agent_name}_node',
        parameters=[agent_config],
        output='screen'
    )
    
    return [env_node, agent_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
"""
        
        launch_path = output_dir / f'{env_name}_{agent_name}.launch.py'
        with open(launch_path, 'w') as f:
            f.write(launch_content)
            
        return str(launch_path) 