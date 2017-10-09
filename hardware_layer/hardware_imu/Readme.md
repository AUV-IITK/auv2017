# Inertial Measurement Unit(IMU)
  An IMU is a sensor which has accelerometer, gyroscope, and magnetometer in it and gives us its acceleration, angular velocity
magnetic field values at that point.   
  Here we are using a [PIXHAWK](https://pixhawk.org), a flight controller which has many features inside it that makes executing
and studying flights with the help from other sensors that can be attached to it. There are many algorithms that runs in it which
processes data outputs from various sensors and use the processed data for various motions of the flight. Similarly, it has an IMU
as well. It applies [EKF](https://en.wikipedia.org/wiki/Extended_Kalman_filter) algorithm on the IMU's data and gives the output in form of [quateranions](https://en.wikipedia.org/wiki/Quaternion) to us.

  To use PIXHAWK with ROS install **Mavros** package from the [Readme.md](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb) file. Go down to the **Binary Installation** title and run the command
 ```
 sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
 ```
for Ubuntu **Kinetic**. After this installation plug-in PIXHAWK in your laptop and run the command
```
rosrun mavros mavros_node _fcu_url:=/dev/ttyACM0:921600 _gcs_url:=udp://@172.16.254.1
```
as given in the **PROGRAMS** title of the [Readme.md](https://github.com/mavlink/mavros/blob/master/mavros/README.md#binary-installation-deb) file. You have to take care that your PIXHAWK is connected via **ACM0** port. Otherwise, you hace to change the port **number** accordingly.

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
