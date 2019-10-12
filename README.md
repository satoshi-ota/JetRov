### Add i2c permission to login user

`sudo usermod [UserName] -aG i2c`

`git clone https://github.com/ccny-ros-pkg/imu_tools.git`

`sudo xboxdrv --detach-kernel-driver --silent`

### Setting Arduino buffer size

Refer to this site.
https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/

```
#elif defined(__AVR_ATmega2560__)

  typedef NodeHandle_<ArduinoHardware, 15, 15, 512, 1024> NodeHandle;
```

```
namespace ros {

  using rosserial_msgs::TopicInfo;

  /* Node Handle */
  template<class Hardware,
           int MAX_SUBSCRIBERS=25,
           int MAX_PUBLISHERS=25,
           int INPUT_SIZE=512,
           int OUTPUT_SIZE=1024>
  class NodeHandle_ : public NodeHandleBase_
  {
```
