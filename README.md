# Vision-detect-and-track
## 1. Description
The main  objective of this repository is to provide a set of computer vision tools to detect and track objects depending on certain characteristics, however, it was particularly designed to identify a soccer ball inside a soccer field. These systems where developed using the Robotic Operating System and the Open source Computer Vision library for C++. The tools implemented in the repository are:
* Kalman Filter + HAAR/LBP Cascade
* Particle Filter + Color Detection
* Kalman Filter + Color Detection
* SURF
* Dynamixel Servomotors Motion

![Image of kalman filter detection](https://github.com/marcovc41/vision-detect-and-track/blob/master/read_img/captura_1.png)

## 2. Requirements

#### Software

1. [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation)
2. [OpenCV3 for ROS](http://wiki.ros.org/vision_opencv)
3. [Dynamixel libraries for ROS](https://github.com/aaceves/example_dynamixel)

    And if you want to train your own detection cascade:

4. [Computer Vision toolbox for MATLAB](https://www.mathworks.com/products/computer-vision.html)

#### Hardware

1. Camera

    In order to use Dynamixel motors, the following components are needed:
    
2. Dynamixel motors
3. Switched Modulated Power Supplier
4. U2D2-power-hub / USB2Dynamixel + SMPS2Dynamixel

For more details check this readme: https://github.com/aaceves/example_dynamixel

## 3. Installation

#### Installation Guide

Once the requirements have been met and the [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) have been built, the next step is to clone and build this repository using the following commands:
```
cd catkin_ws/src
git clone https://github.com/marcovc41/vision-detect-and-track.git
cd ..
catkin_make
source devel/setup.bash
```
Depending on how you set your workspace, `catkin build` may be used instead of `catkin_make`.

If all the steps where successfully completed,  no errors should appear after using `catkin_make` or `catkin build` to build the code. If you have any doubt about using ROS, please check [ROS documentation](http://wiki.ros.org/) and follow the tutorials. 
Note: A dynamixel library error would appear if you haven't set your library yet, if you are not interested in using Dynamixel motors, please errase the corresponding lines of the track program in the package CMakeLists.txt and delete "dynamixel_sdk" that appears inside "find_package", then build your workspace again and the problem should be fixed.

Before running the programs, some modifications to the code have to be made because of the changes in directories for your machine. 
For the SURF node: Open /vision_tools/src/SURF.cpp with your favorite text editor. Look for the `path_image` variable assignment inside the main function, change it for your corresponding path, in my case it is `path_image= "/home/marco/catkin_ws/src/vision_tools/img/cuad2.png";`
#### Using the programs

In order to verify that the installation was successful, run the following command:
```
roscore
cntrl+shift+T
rosrun vision_tools particlefilter
```
Then a screen with your webcam images and the particle filter must appear.
The way to run each of the ROS nodes is described below:

###### Kalman Filter + HAAR/LBP Cascade
```
rosrun vision_tools detect <debugger mode (0/1)> [path to video]

Examples:
For offline debugger mode:  rosrun vision_tools detect 1 '/home/marco/catkin_ws/src/vision_tools/img/prueba1.mp4'
For realtime debugger mode: rosrun vision_tools detect 1
```
###### Kalman Filter + Color Detection
```
rosrun vision_tools kalmanfilter <debugger mode (0/1)> [path to video]

Examples:
For offline mode: rosrun vision_tools detect 0
```
###### Particle Filter + Color Detection
```
rosrun vision_tools particlefilter
```
###### SURF
```
rosrun vision_tools SURF 
```
###### Dynamixel motors
```
rosrun vision_tools track
```
## 4. Explanation
#### Kalman Filter
The [Kalman Filter](https://www.mathworks.com/videos/series/understanding-kalman-filters.html) algorithm was implemented in code because of its good tracking, noise rejection and observer properties. In these package, it was combined with two different detection methods: Color Detection and a Cascade Object Detector. The latest version of the program is the detect node, which implements some corrections on minor bugs and optimizes the code to increase the performance. 
In order to detect and track the soccer ball, a Linear State Space Gaussian Model of an uniform acceleration particle was implemented.
