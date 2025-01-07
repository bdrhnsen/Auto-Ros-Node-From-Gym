# Core API Documentation

## GymEnvWrapper

Base wrapper class for Gym environments that handles ROS2 conversions.

### Methods

#### `__init__(env_id: str, env_config: Optional[Dict] = None)`
Initialize the environment wrapper.

**Parameters:**
- `env_id`: The Gymnasium environment ID
- `env_config`: Optional configuration dictionary for the environment

#### `reset() -> Tuple[Any, Dict]`
Reset the environment.

**Returns:**
- Tuple containing initial observation and info dictionary

#### `step(action: Any) -> Tuple[Any, float, bool, bool, Dict]`
Take a step in the environment.

**Returns:**
- Tuple containing (observation, reward, terminated, truncated, info)

#### `close() -> None`
Close the environment.

## ROS2GymNode

Base ROS2 node class for Gym environment interaction.

### Methods

#### `__init__(node_name: str, env_wrapper: GymEnvWrapper)`
Initialize the ROS2 node.

**Parameters:**
- `node_name`: Name of the ROS2 node
- `env_wrapper`: Instance of GymEnvWrapper 