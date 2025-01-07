# Generic observation message that can handle different types of observations
uint8 DISCRETE=0
uint8 CONTINUOUS=1
uint8 IMAGE=2
uint8 DICT=3

uint8 type

# For discrete and continuous observations
float32[] continuous_data
int32[] discrete_data

# For image observations
uint32 height
uint32 width
uint32 channels
uint8[] image_data

# For dictionary observations
string[] keys
float32[] values 