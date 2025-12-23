# Only a Spoonful
This device is a self-balancing eating utensil designed to assist those with Parkinson's disease and wrist tremors. It utilizes an MPU6050 IMU (featuring a 3-axis accelerometer and gyroscope) as well as an SG90 servo motor to control the head of the utensil. An ESP32-S3 microcontroller is used to run the sensor fusion algorithm, which combines the accelerometer and gyroscope data to determine the tilt angle, as well as the PID controller for precise stabilization to counteract rapid movements. It also features a toggle switch and an indicator LED to enable or disable stabilization, as well as a 5V battery to power the ESP32 via USB. The design utilizes 3D-printed parts to keep the device lightweight and features two rings at the base to provide a more secure grip for the user.

This project was done for the 2025 DO.IT hardware hackathon, which took place over the course of a weekend. The project placed first overall in the competition.

<img src="https://i.imgur.com/0oOeLcy.jpeg" height="80%" width="80%" alt="Disk Sanitization Steps"/>
<img src="https://i.imgur.com/SLklWq0.jpeg" height="80%" width="80%" alt="Disk Sanitization Steps"/>
<img src="https://i.imgur.com/yXbSmZI.jpeg" height="80%" width="80%" alt="Disk Sanitization Steps"/>
