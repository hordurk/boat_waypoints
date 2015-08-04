cmake_minimum_required(VERSION 2.8.3)
project(boat_waypoints)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs geometry_msgs tf actionlib actionlib_msgs usc_asv_msgs std_msgs message_generation)

add_message_files(
    DIRECTORY msg
    FILES Status.msg Debug.msg
)

generate_messages(
    DEPENDENCIES std_msgs usc_asv_msgs
)

catkin_package(
    DEPENDS rospy sensor_msgs geometry_msgs tf actionlib actionlib_msgs usc_asv_msgs python-numpy python-scipy python-pyproj std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
