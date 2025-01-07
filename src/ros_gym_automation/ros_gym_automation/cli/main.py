#!/usr/bin/env python3
import click
import os
from pathlib import Path
from ..generators.env_parser import GymEnvParser
from ..generators.node_generator import EnvNodeGenerator
from ..generators.agent_generator import AgentNodeGenerator
from ..generators.config_generator import EnvConfigGenerator
from ..generators.agent_config_generator import AgentConfigGenerator


@click.group()
def cli():
    """ROS-Gym Automation CLI tool for generating ROS2 nodes from Gym environments."""
    pass


@cli.command()
@click.argument('env_id')
@click.argument('output_dir')
@click.option('--package-name', '-p', default='ros_gym_automation',
              help='Name of the ROS2 package')
@click.option('--config', '-c', type=click.Path(exists=True),
              help='Path to environment configuration file')
@click.option('--agent-name', '-a',
              help='Name of the agent to generate launch file with')
@click.option('--agent-config', '-ac', type=click.Path(exists=True),
              help='Path to agent configuration file for launch file')
def generate_env(env_id: str, output_dir: str, package_name: str, config: str,
                agent_name: str = None, agent_config: str = None):
    """Generate a ROS2 node for a Gym environment."""
    try:
        # Load configuration if provided
        env_config = None
        if config:
            import yaml
            with open(config, 'r') as f:
                env_config = yaml.safe_load(f)
        
        # Generate environment node
        generator = EnvNodeGenerator()
        node_path = generator.generate_node(env_id, package_name, output_dir, env_config)
        
        # Generate configuration files
        config_generator = EnvConfigGenerator()
        config_files = config_generator.generate_config(
            GymEnvParser.get_env_specs(env_id),
            output_dir,
            env_id
        )
        
        click.echo(f"Successfully generated environment node: {node_path}")
        click.echo("Configuration files:")
        for config_type, path in config_files.items():
            click.echo(f"  {config_type}: {path}")
            
        # Generate launch file if agent information is provided
        if agent_name and agent_config:
            from ..generators.launch_generator import LaunchGenerator
            launch_path = LaunchGenerator.generate_launch_file(
                package_name,
                env_id.replace('-', '_'),
                agent_name,
                os.path.basename(config) if config else 'default_env_config.yaml',
                os.path.basename(agent_config),
                output_dir
            )
            click.echo(f"Generated launch file: {launch_path}")
            
    except Exception as e:
        click.echo(f"Error generating environment node: {e}", err=True)
        raise click.Abort()


@cli.command()
@click.argument('agent_name')
@click.argument('output_dir')
@click.option('--package-name', '-p', default='ros_gym_automation',
              help='Name of the ROS2 package')
@click.option('--env-id', '-e', required=True,
              help='Gym environment ID for space specifications')
@click.option('--config', '-c', type=click.Path(exists=True),
              help='Path to agent configuration file')
def generate_agent(agent_name: str, output_dir: str, package_name: str,
                  env_id: str, config: str):
    """Generate a ROS2 node for an RL agent."""
    try:
        # Get environment specifications
        env_specs = GymEnvParser.get_env_specs(env_id)
        
        # Load configuration if provided
        agent_config = None
        if config:
            import yaml
            with open(config, 'r') as f:
                agent_config = yaml.safe_load(f)
        
        # Generate agent node
        generator = AgentNodeGenerator()
        node_path = generator.generate_node(
            agent_name,
            package_name,
            env_specs['observation_space'],
            env_specs['action_space'],
            output_dir,
            agent_config
        )
        
        # Generate configuration files
        config_generator = AgentConfigGenerator()
        config_files = config_generator.generate_config(
            agent_name,
            env_specs['observation_space'].__dict__,
            env_specs['action_space'].__dict__,
            output_dir,
            agent_config
        )
        
        click.echo(f"Successfully generated agent node: {node_path}")
        click.echo("Configuration files:")
        for config_type, path in config_files.items():
            click.echo(f"  {config_type}: {path}")
            
    except Exception as e:
        click.echo(f"Error generating agent node: {e}", err=True)
        raise click.Abort()


if __name__ == '__main__':
    cli() 