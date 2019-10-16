# Skystone19-20

Code for Dutton Christian School's Middle School Robotics team for the 2019-2020 FTC Skystone challenge. This repository will facilitate work among all programming mentors and team members.

## Code Goals:
* Abstract bot functionality into a DefenderBot class so that multiple specific OpModes can reuse code.
* Allow for maximum flexibility in testing by abstracting configuration and setup information into easily swappable components.
* Ensure that "tweaking" functionality can be done without rewriting core functionality.
* Provide a library of easy to use tools for common situations (e.g. perform a robot function until time expires or some other condition happens).

## To Do List:
* Implement STOP ALL MOTORS function
* Continue to properly abstract drive functions.
* Implement ability to drive a particular distance (rather than time), with drive time calculated based on time/distance constants.
* Get a functional "wait" method that can be interrupted by sensors.
* Implement additional sensor types, such as magnetic switch and color/distance sensor.
* Implement ability to drive Hex motors by encoder distance.
* Implement manipulator motors

## In Progress
* Implement a configuration class. This could include values for motor drive times, as well as names for port mappings. This class would be instantiated in the OpMode class and passed to the DefenderBot class. This will allow us to keep the Bot code clean and minimally changeable, but let us quickly swap out and test functionality by using different configuration files.
* Developed op mode (JuniorIterativeTeleOp) for basic driving with controller

## Done
* Created motor subclass that allows "forward" and "reverse" to be established in configuration file--for those times when motors are mounted backward.
* Move motor forward/backward names to configuration