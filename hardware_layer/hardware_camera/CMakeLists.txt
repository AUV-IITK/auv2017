cmake_minimum_required(VERSION 2.8)
project(hardware_camera)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
)

find_package(OpenCV REQUIRED)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(vid_pub src/vid_pub.cpp)
target_link_libraries(vid_pub ${catkin_LIBRARIES})
target_link_libraries(vid_pub ${OpenCV_LIBS})

# add the publisher example
add_executable(image_listener src/image_listener.cpp)
target_link_libraries(image_listener ${catkin_LIBRARIES})
target_link_libraries(image_listener ${OpenCV_LIBS})