cmake_minimum_required(VERSION 2.8.3)
project(vision_tools)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dynamixel_sdk
	OpenCV
)

add_compile_options(-std=c++11)

catkin_package(LIBRARIES OpenCV
               CATKIN_DEPENDS roscpp)

include_directories( ${OpenCV_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS})

add_executable(particlefilter src/Particle1.cpp)
target_link_libraries(particlefilter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(kalmanfilter src/KalmanFilter.cpp)
target_link_libraries(kalmanfilter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(track src/TrackBall.cpp)
target_link_libraries(track ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${dynamixel_sdk_LIBRARIES})

add_executable(SURF src/SURF.cpp)
target_link_libraries(SURF ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detect src/DetectBall.cpp)
target_link_libraries(detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
