cmake_minimum_required(VERSION 3.0.2)
project(galaxy_camera)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    image_transport
    camera_info_manager
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME}_node
    src/main.cpp
    src/GxCamera.cpp
    )

target_link_libraries(${PROJECT_NAME}_node
    gxiapi
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    )
