# Environment status message
bool done  # Episode ended naturally
bool truncated  # Episode was truncated
uint32 episode_step  # Current step in episode
uint32 total_steps  # Total steps across all episodes
float32 total_reward  # Cumulative reward for current episode

# Additional info dictionary
string[] info_keys
string[] info_values 