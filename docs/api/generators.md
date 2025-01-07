# Generator API Documentation

## EnvNodeGenerator

Generator for ROS2 environment nodes.

### Methods

#### `generate_node(env_id: str, package_name: str, output_dir: str, env_config: Dict[str, Any] = None) -> str`
Generate a ROS2 node for a Gym environment.

**Parameters:**
- `env_id`: Gymnasium environment ID
- `package_name`: Name of the ROS2 package
- `output_dir`: Directory where to save the generated node
- `env_config`: Optional environment configuration

**Returns:**
- Path to the generated node file

## AgentNodeGenerator

Generator for ROS2 agent nodes.

### Methods

#### `generate_node(agent_name: str, package_name: str, observation_space: SpaceSpec, action_space: SpaceSpec, output_dir: str, agent_config: Dict[str, Any] = None) -> str`
Generate a ROS2 node for an RL agent.

**Parameters:**
- `agent_name`: Name of the agent
- `package_name`: Name of the ROS2 package
- `observation_space`: Specification of the observation space
- `action_space`: Specification of the action space
- `output_dir`: Directory where to save the generated node
- `agent_config`: Optional agent configuration

**Returns:**
- Path to the generated node file 