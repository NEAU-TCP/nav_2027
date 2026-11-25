std_msgs/Header header
ros_interfaces/PositionCommand[] cmds
uint32 mpc_horizon
uint8 command_flag
uint8 NORMAL_COMMAND = 1
uint8 BLOCK_COMMAND = 137
float64[3] kx
float64[3] kv
uint32 trajectory_id
