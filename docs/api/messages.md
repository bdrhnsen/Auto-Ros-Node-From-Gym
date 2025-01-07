# Message Types and Services Documentation

## Message Types

### Observation
Message type for environment observations. 

For Box space (continuous observations)
float32[] data # Flattened observation array
int32[] shape # Original observation shape
For Discrete space
int32 data # Discrete observation value


### Action
Message type for agent actions.

For Box space (continuous actions)
float32[] data # Action values array
int32[] shape # Action shape
For Discrete space
int32 data # Discrete action value


### Reward
Message type for environment rewards.
float32 data # Reward value for the current step

### EnvStatus
Message type for environment status information.

bool ready # Whether environment is ready for interaction
string state # Current environment state (e.g., "running", "reset", "done")
bool terminated # Whether episode has terminated
bool truncated # Whether episode was truncated

## Services

### ResetEnv
Service for resetting the environment.

Request
bool seed_enabled # Whether to use a specific seed
int32 seed # Seed value if enabled
---
Response
bool success # Whether reset was successful
string message # Status message
float32[] initial_observation # Initial observation after reset

### ConfigureEnv
Service for configuring the environment.

Request
string config_yaml # Environment configuration in YAML format
---
Response
bool success # Whether configuration was successful
string message # Status or error message