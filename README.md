----------------------------------------------------------- Arduino-Robot Car ------------------------------------------------------------
#
##
###
####
#####
								HARDWARE

-- Arduino mega 2560 controller board;//
-- V5.0 extension board;
-- L298N motor driver board;
-- Hc-08 Ble module;
-- GY-521 module;
-- Ultrasonic Sensor;
-- Servo motor;
-- Stepper motor
-- ULN2003 stepper motor driver module;
-- Motor X4;
-- Cell box;
-- 18650 battery X2;
-- Wheel X4;



								SOFTWARE

The code is divided into 5 sections:
-- Hardware Section: functions here directly communicate with the arduino motherboard. Their main purpose is to activate the robot parts(motors, ultrasonic sensor).

-- Setup Section: this is where the setup happens. 
   	The robot connects to all the hardware devices, inizializes the gyroscope and calculates  the varaition of the acceleration so it
	can find the average and the standard deviation when is still and when is moving. This in usefull controll if the robot  is stuck.

-- Gyroscope Section: this is the section that gets the accelleration from the gyroscope, caculates its variation and determines if the 
	robot is stuck by confronting it with the values found in the Setup Section.
	To be sure little vibration don't influence too much the robot behaviour the function considers the average of the last ten
	variation values found. 

-- Obstacle Control Section: this part controls there are no obstacles in front of the robot. If yes it stops it and controls if there are 
	obstacle on his left or right and than turns or goes eventually back. 

-- Main Control Section: this is the main body of the robot. It decides the next robot action and sets the time  for every action and 
	invokes the obsatcle and gyroscope control functions.

NOTE: The robot is based on the ELEGOO UNO R3 Project Smart Robot Car Kit V 3.0 Plus and can be controlled with the ELEGOO BLE Tool APP from APP Store and Google Play by connecting it to the bluetooth module on the robot(HC-08).
