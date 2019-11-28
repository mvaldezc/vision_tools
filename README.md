# Vision-detect-and-track
## 1. Description
The main  objective of this repository is to provide a set of computer vision tools to detect and track objects depending on certain characteristics, however, it was particularly designed to identify a soccer ball inside a soccer field. These systems where developed using the Robotic Operating System and the Open source Computer Vision library for C++. The tools implemented in the repository are:
* Kalman Filter + HAAR/LBP Cascade
* Particle Filter + Color Detection
* Kalman Filter + Blob Color Detection
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

Once the requirements have been met and the [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) have been built, the next step is to clone and build this repository, using the following commands:
```
cd catkin_ws/src
git clone

```
