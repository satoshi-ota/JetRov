std_msgs/Header header

#control frequency
#float64 ros_control_loop_freq

#robot move direction
marin_msgs/MarinMoveDirection move_direction

#emergency command
bool emergency_stop

#target velocity command
#geometry_msgs/Twist tgt_cmd_vel

#control command
int16 tgt_pulse
float64 tgt_steer
