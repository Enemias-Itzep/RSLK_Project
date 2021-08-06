PLEASE READ THIS FILE BEFORE USING THIS PROGRAM!

Hardware Information:
This program is intended to be run on two TI MSP432P401R launchpads using two CC3100 BoosterPacks. One launchpad should be connected to a
fully assembled daughter board v2.0, designed for Dr. Yier Jin's Microprocessor Applications II course at the University of Florida.
The other launchpad should be connected to a fully assembled TI RSLK MAX ROMI Chassis Board.

Compiling and Running:
The program was compiled and tested using C standard C11 within Code Composer Studio (CCS). In order to run this code on your devices,
open CCS, create a new project, and drop in the code files and folders downloaded from GitHub without making any changes to the file structure.
Then, navigate to File=>C/C++=>CCS Project Settings and select the settings file downloaded from the GitHub. The same project can be compiled
and uploaded onto the controller launchpad and the RSLK launchpad without changes to the code, as controller and RSLK settings are determined during run time using 
the two buttons on the sides of the launchpad.

Usage Information:
Take care to press the correct buttons on the controller and RSLK when launching the program. Press the button labelled
S1 on the controller's launchpad (the daughter board launchpad) and press S2 on the RSLK's launchpad to correctly start the program. Incorrect usage may result in hardware damage,
so use with caution.

Controls and Functionality:
In this program, the controller hosts a WiFi connection between the controller and the RSLK that sends JoyStick and bumper information back and forth.
The controller reads inputs from the JoyStick and sends them to the RSLK's launchpad, which uses the inputs to control the motors using a TimerA module. Holding up on the JoyStick
causes the RSLK to move forward, holding back makes it move backwards, holding left makes it turn left, and holding right makes it turn right. Note that the code that drives the motors
can only select one direction at a time. A top-down picture of the RSLK is displayed on the LCD during use. Red dots appear near the position of each bumper when they are pressed, indicating where a collision has occurred.
The red dots are updated once a new collision occurs.

--Ryan Roth, Enemias Itzep