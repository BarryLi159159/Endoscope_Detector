Endoscope Measurement and Straightening System
This repository contains the computer vision and control code for an autonomous system that measures and straightens medical-grade endoscopes. Developed as part of an MQP at WPI in collaboration with Henke Sass Wolf of America, this project aims to improve accuracy and efficiency in endoscope repair.

Features
Measurement: Detects bends using OpenCV-based computer vision.
Straightening: Uses a cold rolling process to correct bends autonomously.
Precision: Measurement accuracy of ±0.5 mm; straightening accuracy of ±1.0 mm.
Repository Contents
cv_code/: Python scripts for bend detection and runout calculation.
control_code/: Arduino scripts for motor control during straightening.
Setup and Usage
Hardware: Assemble with a 4K camera, NEMA stepper motors, TB6600 drivers, and Arduino Mega.
Software: Install Python 3.8, Arduino IDE, and dependencies (opencv-python, numpy, serial).
Execution:
Run cv_code/ for measurement.
Use control_code/ to straighten endoscopes based on detected bends.
Contributors
Chenhao Li: Computer Vision Lead
Team Members: Abigail Clemence, Nikita Igoshin, Praniva Pradhan, Jessica Rhodes, George Shelton
Advisor: Prof. Pradeep Radhakrishnan, Henke Sass Wolf of America
