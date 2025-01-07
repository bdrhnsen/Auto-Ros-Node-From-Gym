# Generic action message that can handle different types of actions
uint8 DISCRETE=0
uint8 CONTINUOUS=1
uint8 MULTI_DISCRETE=2
uint8 MULTI_BINARY=3

uint8 type

# For discrete actions
int32 discrete_value

# For continuous actions
float32[] continuous_values

# For multi-discrete actions
int32[] multi_discrete_values

# For multi-binary actions
bool[] binary_values 