std_msgs/Header header

#Encoder status
int16 pulse_count

#IMU status
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration

#robot condition
float64 battely_voltage
bool no_battery
