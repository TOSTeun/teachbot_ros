# TeachbotState.msg
# Complete state of the TOS Teachbot including joints, TCP, and controls

# Header with timestamp
std_msgs/Header header

# Joint angles in degrees (6 DOF)
float64[6] joint_angles_deg

# TCP position from forward kinematics (mm)
float64 tcp_x
float64 tcp_y
float64 tcp_z

# TCP orientation (degrees)
float64 tcp_rx
float64 tcp_ry
float64 tcp_rz

# Gripper/control inputs
int32 pot_raw           # Raw potentiometer value (0-1023)
int32 pot_percent       # Mapped potentiometer percentage (0-100)
bool btn1               # Button 1 state
bool btn2               # Button 2 state

# Encoder status per joint
bool[6] encoder_errors
bool[6] encoder_warnings
float64[6] encoder_frequencies

# Model information
string robot_model
