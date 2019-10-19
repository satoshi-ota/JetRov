#ifndef JETROV_MSGS_DEFAULT_TOPICS_H
#define JETROV_MSGS_DEFAULT_TOPICS_H

namespace jetrov_msgs{
namespace default_topics{

//command
static constexpr char COMMAND_VELOCITY[] = "/command/velocity";
static constexpr char COMMAND_JOY[] = "/command/joy";

static constexpr char RAW_IMU[] = "/imu/data_raw";

// status
static constexpr char STATUS_PULSE_COUNT[] = "/status/pulse_count";
static constexpr char STATUS_IMU[] = "/status/imu";
static constexpr char STATUS_ODOMETRY[] = "/status/odometry";

} //namespace default_topic
} //namespace jetrov_msgs

#endif //JETROV_MSGS_DEFAULT_TOPIC_H
