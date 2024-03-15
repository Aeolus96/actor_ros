Header header                   # Optional - Used for external logging

# Node Status
bool is_simulated               # Configured for a simulator
bool is_autonomous              # Optional Status indicator - set externally
bool is_tele_operated           # Optional Status indicator - set externally

# Vehicle Status
float32 accelerator_percent     # Percentage of maximum pedal position
float32 brake_percent           # Percentage of maximum brake torque
string gear                     # String representation of gear state
float32 steering_wheel_angle    # Direct steering wheel angle in degrees
float32 road_angle              # Angle from car centerline to drive wheel vector in degrees
float32 speed                   # Speed in miles per hour

# ROS Control Status
bool is_enabled                 # Actor ROS to hardware control enabled
float32 speed_limit             # software capped maximum speed
float32 requested_speed         # Requested speed in mph
float32 requested_road_angle    # Requested road angle in degrees

# E-Stop Status
bool estop_state                # E-Stop state
bool estop_physical_button      # E-Stop physical button
bool estop_wireless_button      # E-Stop wireless button
bool estop_software_button      # E-Stop software button