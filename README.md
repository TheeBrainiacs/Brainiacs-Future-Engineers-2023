Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2023.

## Content

//REVIEW THIS//
* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.


## Introduction
//REVIEW THIS//
_ This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._

* The code for the self-driving autonomous race robotic car consists of several modules that are related to the electromechanical components of the vehicle. The Jetson Nano module runs the computer vision algorithms that analyze camera data to detect obstacles in the car's path. The Arduino Mega and Arduino Nano modules are responsible for controlling the motors, servos, and sensors that enable the car to move and navigate. The MPU6050 gyroscope sensor is used for PID control, which helps to maintain the car's stability and balance while driving. The TCS34725 color sensor detects colors in the environment, which can be used to help the car navigate.

* To build and compile the code for the self-driving autonomous race robotic car, the Arduino IDE was used to program the Arduino boards using the C++ language. The Jetson Nano module used Linux and the Python language to perform computer vision tasks. Manual compiling was done using the Linux command prompt on the Jetson Nano. The code for the Arduino boards was compiled into a binary file that could be uploaded to the boards using a USB cable or a wireless connection. The code for the Jetson Nano was written in Python and compiled manually using the Linux command prompt. Once the code was compiled, it could be uploaded to the controllers and executed to control the sensors and actuators and enable the car to drive autonomously. It is important to test the car in a safe environment and make adjustments to the code as needed to improve performance and reliability.
