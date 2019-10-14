# Skystone19-20

Code for Dutton Christian School's Middle School Robotics team for the 2019-2020 FTC Skystone challenge. This repository will facilitate work among all programming mentors and team members.

## Code Goals:
* Abstract bot functionality into a DefenderBot class so that multiple specific OpModes can reuse code.

## To Do List:
* Implement a configuration class. This could include values for motor drive times, as well as names for port mappings. This class would be instantiated in the OpMode class and passed to the DefenderBot class. This will allow us to keep the Bot code clean and minimally changeable, but let us quickly swap out and test functionality by using different configuration files.
* Continue to properly abstract drive functions.
* Implement ability to drive a particular distance (rather than time), with drive time calculated based on time/distance constants.
* Get a functional "wait" method that can be interrupted by sensors.
* Implement additional sensor types, such as magnetic switch and color/distance sensor.
* Implement ability to drive Hex motors by encoder distance.
* Implement manipulator motors
* Move motor forward/backward names to configuration

## In Progress
* Developed op mode (JuniorIterativeTeleOp) for basic driving with controller