# Vision-detect-and-track
## 1. Description
<img src="read_img/captura_1.png" align="right" width="312" height="239"/>
<p align="justify"> 
The main  objective of this repository is to provide a set of computer vision tools to detect and track objects depending on certain characteristics, however, it was particularly designed to identify a soccer ball inside a soccer field. These systems where developed using the Robotic Operating System and the Open Source Computer Vision library for C++11. The tools implemented in the repository are:
</p>

* Kalman Filter + HAAR/LBP Cascade
* Particle Filter + Color Detection
* Kalman Filter + Color Detection
* SURF
* Camera Angular Position Control (Dynamixel Servomotors)



## 2. Requirements
#### Software

1. [ROS1 Kinetic Kame](http://wiki.ros.org/kinetic)
2. [OpenCV3 for ROS](https://github.com/aaceves/opencv_tutorial)
3. [Dynamixel libraries for ROS](https://github.com/aaceves/example_dynamixel)

    And if you want to train your own detection cascade:

4. [Computer Vision toolbox for MATLAB](https://www.mathworks.com/products/computer-vision.html)

#### Hardware

1. Webcam

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

If all the steps where successfully completed,  no errors should appear after building the code. Note: A dynamixel library error would appear if you haven't set your library yet. If you are not interested in using Dynamixel motors, please errase the corresponding lines of the "track" program in the package CMakeLists.txt and delete the text "dynamixel_sdk" that appears inside "find_package", then build your workspace again and the problem should be fixed.

Before running the programs, some modifications to the code are needed due to the changes in directories for your machine. 

For the SURF node: In the downloaded repository, open `/vision_tools/src/SURF.cpp` with your favorite text editor. Look for the `path_image` variable assignment inside the main function, change it for your corresponding path, in my case it is `path_image= "/home/marco/catkin_ws/src/vision_tools/img/cuad2.png";`. The path that you have to write is the path of the sample image that you will search using the SURF algorithm.

For the detect node: open `/vision_tools/src/DetectBall.cpp` and change the ball.cascade path inside the main function, II section. In my case it looks like `if( !ball_cascade.load("/home/marco/catkin_ws/src/vision_tools/cascade/ballDetector.xml" ))`, you have to provide the complete path of the cascade detector trained file that is located in the cloned repository `/vision_tools/cascade/ballDetector.xml`. After these steps, build the packages again and you will be able to use the programs.

#### Using the programs

In order to verify that the installation was successful, run the following command:
```
roscore
cntrl+shift+T
rosrun vision_tools particlefilter
```
Then a screen with your webcam image and the particle filter must appear.
The way to run each of the ROS nodes is described below:

###### Kalman Filter + HAAR/LBP Cascade
```
rosrun vision_tools detect <debugger mode (0/1)> [path to video]
```
###### Kalman Filter + Color Detection
```
rosrun vision_tools kalmanfilter <debugger mode (0/1)> [path to video]
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
#### ROS

<img src="read_img/kinetic.png" align="right" width="157" height="130"/>

The [Robot Operating System (ROS)](https://www.ros.org/) is a flexible modular framework for writing robot software. It is a set of open source software libraries and tools that aim to simplify developing robot aplications. This ecosystem provides services such as hardware abstraction, low-level device control, package management, communications infrastructure, diagnostics, pose estimation, localization, mapping, navigation, GUI, simulation, computer vision, etc. If you want to know more about using ROS, please check [ROS1 wiki](http://wiki.ros.org/ROS).

Due to the amount of packages available, Kinetic Kame [ROS distribution](http://wiki.ros.org/Distributions) is used for this package. However, it is getting older and it will reach its end of life starting 2021, therefore a migration from Kinetic Kame to Melodic Morenia must be seeked soon.

#### OpenCV

<img src="read_img/opencv.png" align="right" width="100" height="123"/>

The [Open Source Computer Vision Library](https://opencv.org/) is a set of software functions that provide a common infrastructure to computer vision and machine learning applications with over 2500 optimized algorithms. These algorithms can be used to detect faces, identify objects, track moving objects, image processing, etc. OpenCV3 is the default version for ROS Kinetic and is already system dependency. In order to install it check this [repository](https://github.com/aaceves/opencv_tutorial) and [ROS documentation](wiki.ros.org/vision_opencv). To about the different functions available and application examples, check the [tutorials](https://docs.opencv.org/3.3.1/d9/df8/tutorial_root.html)

#### Kalman Filter
The [Kalman Filter](https://www.mathworks.com/videos/series/understanding-kalman-filters.html) algorithm was implemented in code because of its good tracking, noise rejection and observer properties. In this package, it was combined with two different detection methods: Color Detector and a Cascade Object Detector. The latest version of the program is the detect node (cascade detector), which implements some corrections on minor bugs and optimizes the code to increase the performance. 
In order to detect and track the soccer ball, a Discrete-Time Linear Gaussian State Space Model of a particle with uniform acceleration was used.
![Image of State Space Model](https://github.com/marcovc41/vision-detect-and-track/blob/master/read_img/kalman1.PNG)

#### Particle Filter


#### Cascade Object Detector



(https://www.mathworks.com/videos/series/understanding-kalman-filters.html)
https://www.mathworks.com/help/vision/examples/using-kalman-filter-for-object-tracking.html
https://www.ros.org/
http://wiki.ros.org/ROS


Examples:
For offline debugger mode:  rosrun vision_tools detect 1 '/home/marco/catkin_ws/src/vision_tools/img/prueba1.mp4'
For realtime debugger mode: rosrun vision_tools detect 1

Examples:
For offline mode: rosrun vision_tools kalmanfilter 0
