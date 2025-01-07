# ROS-Gym Automation

A framework for automatically generating ROS2 nodes from Gymnasium environments.

## Features

- Automatic ROS2 node generation for Gym environments
- Support for both discrete and continuous action spaces
- Support for vector and image-based observations
- Configurable QoS settings
- ROS2 lifecycle node implementation
- Launch file generation
- Environment parameter configuration

## Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- Gymnasium
- numpy

## Installation

# Clone the repository
git clone https://github.com/bdrhnsen/Auto-Ros-Node-From-Gym.git
cd ros-gym-automation

# Create and activate virtual environment
python -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Build ROS2 package
colcon build
source install/setup.bash

## Quick Start

# Generate environment node
ros-gym-gen env CartPole-v1 my_workspace

# Generate agent node
ros-gym-gen agent my_agent my_workspace --env-id=CartPole-v1

# Launch nodes
ros2 launch my_package cartpole_v1_my_agent.launch.py

## Project Structure

ros_gym_automation/
├── docs/                    # Documentation
│   ├── api/                # API documentation
│   ├── usage.txt           # Usage guide
│   └── installation.txt    # Installation guide
├── examples/               # Example implementations
│   ├── cartpole_example/
├── src/                    # Source code
│   └── ros_gym_automation/
│       ├── core/          # Core framework classes
│       ├── generators/    # Node generators
│       ├── utils/        # Utility functions
│       └── templates/    # Jinja2 templates
└── tests/                 # Test suite

## Documentation

### Core Components

1. Environment Wrapper
   - Handles Gym environment initialization
   - Manages environment lifecycle
   - Provides ROS2 interface conversion

2. Node Generators
   - Environment node generator
   - Agent node generator
   - Launch file generator
   - Configuration generator

3. Message Types
   - Observation messages
   - Action messages
   - Reward messages
   - Environment status messages

4. Services
   - Reset service
   - Configure service
   - Environment control services

### Usage Examples

1. Basic Environment Node:

   # Generate and run CartPole environment
   ros-gym-gen env CartPole-v1 my_workspace
   ros2 run my_package cartpole_v1_node

2. Custom Configuration:

   # Create config file
   environment:
     render_mode: 'rgb_array'
     max_episode_steps: 1000

   # Generate with config
   ros-gym-gen env CartPole-v1 my_workspace --config=config.yaml

3. Agent Node:

   # Generate agent node
   ros-gym-gen agent my_agent my_workspace \
       --env-id=CartPole-v1 \
       --config=agent_config.yaml

4. Launch File:

   # Generate both nodes with launch file
   ros-gym-gen env CartPole-v1 my_workspace \
       --agent-name=my_agent \
       --agent-config=agent_config.yaml

### Configuration

1. Environment Configuration:
   - Render mode
   - Episode limits
   - Environment parameters
   - QoS settings

2. Agent Configuration:
   - Model parameters
   - Observation processing
   - Action generation
   - Training settings

3. ROS2 Configuration:
   - Node names
   - Topic names
   - QoS profiles
   - Namespace settings

## Testing

# Run all tests
python -m pytest tests/

# Run specific test suite
python -m pytest tests/test_core_components.py
python -m pytest tests/test_environment_types.py
python -m pytest tests/test_integration.py

