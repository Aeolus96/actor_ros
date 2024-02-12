Header header                   # Optional - Used for external logging

# States
bool is_simulated               # Configured for a simulator
bool is_autonomous              # Optional Status indicator - set externally
bool is_tele_operated           # Optional Status indicator - set externally

float32 accelerator_percent     # Percentage of maximum pedal position
float32 brake_percent           # Percentage of maximum brake torque
float32 road_angle              # (car centerline <-> drive wheel vector)

float32 speed                   # Speed in miles per hour
float32 speed_limit             # software capped maximum speed

string gear                     # String representation of gear state

# ROS Control
bool is_enabled                 # Actor ROS to hardware control enabled
float32 requested_speed         # Requested speed in mph
float32 requested_road_angle    # Requested road angle in degrees