# AutonomousRCcar
Final Project in KwangwoonUniv.


Autonomous Vehicle using real-time image processing




Using OpenCV Library and Raspberry Pi, Arduino








# Orders

It consists of three orders
1. Real-time image recognition through camera module of Raspberry Pi 3 using VNC(Virtual Networking Computing)
2. Line detection using Hough transformation 
3. Setting the direction of the car according to the angle change of the lane


# Hough transformation using OpenCV

If a straight line is represented by xcosθ + ysinθ = ρ, the same straight line falls to one point in the θ-ρ plane, and therefore a straight line is found by clustering in the θ-ρ plane.

Using  cvHoughLines2() Function in OpenCV Library.This function returns an array of (r, θ) values as pixels in distance and in degrees in radians.




# Autonomous driving vehicle full operation algorithm

- Car speed : 0.2 m/s 
- Car steering angle :  5 º
- Obstacle detecting distance : 18cm

1. Measure the lane angle in real time with VNC through the image processing program.
2. Depending on the measured angle, send 1: left, 2: straight, 3: right to the raspberry pi with file I / O.
3. Raspberry pi sends control information to Arduino for the corresponding signal.
4. Arduino controls the vehicle's steering system to match the signal.
5. When the obstacle is detected by the ultrasonic sensor, the vehicle stops and turns on the LED as a brake light.













# SPEC
l	 Real wheel drive & Front steering systeml	 
2 Motors(DC motor for engine, Servo motor for steering)	
Raspberry Pi 3 with Camera module V2 8Megapixell	 
Arduino Uno & Arduino Nanol	 
9V Battery for Arduino and Servo motorl	 
6V Battery for DC motorl	 
Ultrasonic sensor for stopping  when obstacle is detectedl	
2 Led lights for Brake light



# NOTE
- Install openCV-3.1.0
- Include #include<opencv/cvaux.h>  #include <opencv2/highgui/highgui.hpp>  #include <opencv2/highgui/highgui.hpp>
          #include <opencv2/highgui/highgui.hpp>  #include <opencv2/highgui/highgui.hpp>
          
- Make text file to share with your line detect Project file and Raspberry Pi 3


# Youtube URL
https://youtu.be/9cJNuXgLgDg
