# Inertial Measurement Unit(IMU)
  An IMU is a sensor which has accelerometer, gyroscope, and magnetometer in it and gives us its acceleration, angular velocity
magnetic field values at that point.   
  Here we are using a [PIXHAWK](https://pixhawk.org), a flight controller which has many features inside it that makes executing
and studying flights with the help from other sensors that can be attached to it. There are many algorithms that runs in it which
processes data outputs from various sensors and use the processed data for various motions of the flight. Similarly, it has an IMU
as well. It applies [EKF](https://en.wikipedia.org/wiki/Extended_Kalman_filter) algorithm on the IMU's data and gives the output in form of [quateranions](https://en.wikipedia.org/wiki/Quaternion) to us.

  To use PIXHAWK with ROS install [Mavros](http://wiki.ros.org/mavros) package from [this](https://dev.px4.io/en/ros/mavros_installation.html) site in the src folder of your catkin_ws. While installing, don't run the
last command
```
catkin build
```
Instead use
```
catkin_make_isolated
```
as we use **catkin_make** for compiling our code instead of **catkin build**. Now, connect the PIXHAWK and run the command
```
roslaunch mavros px4.launch
```
Now run the command
```
rostopic list
```
to see all the topics from which data can be extracted from PIXHAWK. Run the command
```
rostopic echo /mavros/data/imu
```
to see the quateranion output from the PIXHAWK. So now make a subscriber of **/mavros/imu/data** topic which has [sensor_msgs/IMU](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
as its message type.
Take the quateranion output and convert it into euclidean form for getting **YAW ANGLE** by using this formula
```
yaw = atan2(2*x*y-2*w*z, 2*w*w+2*x*x-1)
```
## QGround Control
Download [QGround Control](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html) and run it with PIXHAWK attached with your computer.
It is a portal from where you control various settings of the PIXHAWK. To callibrate the magnetometer, accelerometer and gyroscope you can access the
**SENSORS** tab in the settings and callibrate them as per the instructions.
