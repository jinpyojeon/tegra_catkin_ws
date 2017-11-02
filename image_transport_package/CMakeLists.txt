cmake_minimum_required(VERSION 2.8)
project(image_transport_package)

find_package(catkin REQUIRED cv_bridge genmsg compressed_image_transport image_transport sensor_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the publisher 
add_executable(my_publisher src/my_publisher.cpp)
target_link_libraries(my_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the video publisher 
add_executable(video_pub src/video_pub.cpp)
target_link_libraries(video_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the video publisher 
add_executable(video_test src/video_test.cpp)
target_link_libraries(video_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add the subscriber 
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


