                    Endoscope Measurement and Straightening System

This repository contains the computer vision (CV) and control code for the autonomous measurement and straightening system for medical-grade endoscopes. The project was developed as part of a Major Qualifying Project (MQP) at Worcester Polytechnic Institute in collaboration with Henke Sass Wolf of America (HSWoA).

Project Overview
The goal of this project is to replace the current manual process of straightening bent endoscopes with a semi-automated system, improving precision, consistency, and efficiency. The system integrates computer vision for bend detection and a straightening device employing cold rolling for correction.

Key Features:
Measurement Subsystem: Identifies and quantifies bends in endoscopes using OpenCV-based computer vision techniques.
Straightening Subsystem: Corrects detected bends using an autonomous cold rolling process.
Validation: Ensures straightened endoscopes meet the desired specifications with an accuracy of 0.5 mm for measurement and 1.0 mm for straightening.
Repository Contents
1. cv_code/:
Contains Python scripts for endoscope detection and bend measurement using OpenCV.
Features include camera calibration, bend detection, and runout calculation.
2. control_code/:
Arduino scripts for controlling stepper motors and actuators in the straightening subsystem.
Includes integration with Python for combined system functionality.
How to Use
Prerequisites
Hardware:
Camera (4K resolution recommended)
Stepper motors (NEMA 17 and 23) with TB6600 drivers
Cold rolling assembly
Arduino Mega for motor control
Software:
Python 3.8
Arduino IDE
Required Python libraries: opencv-python, numpy, serial
Steps
Setup Hardware:

Assemble the measurement and straightening devices as per the design specifications.
Connect the camera and motors to the Arduino and PC.
Run Measurement Code:

Use the scripts in cv_code/ to detect and quantify bends.
Ensure proper lighting and camera calibration.
Straighten Endoscope:

Use the scripts in control_code/ to operate the straightening device based on the detected bend data.
Validation:

Validate the straightened endoscope using the measurement system.
Results and Performance
Measurement Accuracy: ±0.5 mm
Straightening Time: Under 1 minute per endoscope
Improved precision and reliability compared to manual methods.
Future Work
Integration: Combine the measurement and straightening subsystems for seamless operation.
Enhancements: Improve system robustness and add support for a wider range of endoscope sizes.
Contributors
Chenhao Li: Computer Vision and Integration
Team Members: Abigail Clemence, Nikita Igoshin, Praniva Pradhan, Jessica Rhodes, George Shelton
Advisors: Prof. Pradeep Radhakrishnan, Henke Sass Wolf of America
For more details, refer to the project report: Autonomous Measurement and Straightening of Endoscopes.
