# How to test the new vision code 

1. First build the repo with the new code, for it don't use the command ```.~/catkin_ws/src/auv/utils/build.sh``` just catkin_make so, that it will not give any indentation error.
2. After the code is built then to run the task individually we need to change the code a little, just change ```if(!IP)``` to ```if(1)```.
3. Rebuilt it.
4. For testing the code we need a bag file , so move to the directory where the bagfiles are from where you can play the rosbag files.
5. Open a new terminal and run the command ```roscore``` to start the ros master.
6. Then go back to the bagfiles directory and run ```rosbag play filename.bag```.
7. Rosbag files will publish the images at the right topic .
8. Then workspace directory where the repo is built and source it ```source devel/setup.sh```
9. Then ```rosrun package_name node_name```

[Link for the buoy bagfile](https://drive.google.com/open?id=0ByiJsJ1iI0Y_RVpodlAzdnl0S0U)
[Link for the line bagfile](https://drive.google.com/open?id=0ByiJsJ1iI0Y_OFl2YWpqbUtoRUE)
[Link for the gate bagfile](https://drive.google.com/open?id=0ByiJsJ1iI0Y_UEZlbDR6OFZoS0k)
