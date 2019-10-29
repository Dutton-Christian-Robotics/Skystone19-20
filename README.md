# Skystone19-20

Code for Dutton Christian School's Middle School Robotics team for the 2019-2020 FTC Skystone challenge. This repository will facilitate work among all programming mentors and team members.

## Code Goals:
* Abstract bot functionality into a DefenderBot class so that multiple specific OpModes can reuse code.
* Allow for maximum flexibility in testing by abstracting configuration and setup information into easily swappable components.
* Ensure that "tweaking" functionality can be done without rewriting core functionality.
* Provide a library of easy to use tools for common situations (e.g. perform a robot function until time expires or some other condition happens).

## To Do List:
* Continue to abstract drive functions.
* Implement ability to drive a particular distance (rather than time), with drive time calculated based on time/distance constants.
* Get a functional "wait" method that can be interrupted by sensors.
* Implement additional sensor types, such as magnetic switch and color/distance sensor.
* Implement ability to drive Hex motors by encoder distance.