cmake_minimum_required(VERSION 2.8)
project(image_transporter)

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(vid_pub src/vid_pub.cpp)
target_link_libraries(vid_pub ${catkin_LIBRARIES})

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES})

# add the subscriber example
add_executable(my_subscriber2 src/my_subscriber2.cpp)
target_link_libraries(my_subscriber2 ${catkin_LIBRARIES})


