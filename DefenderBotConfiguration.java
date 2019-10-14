/*
	Speaking from experience, here: in the heat of competition it's easy to start rewriting code to make
	last minute changes. To prevent "tweaking" of code that creates inadvertant problems, and to make
	things more flexible, we use a special configuration class rather than hardcoding values into
	the actual working code of the bot.
*/

package org.firstinspires.ftc.teamcode;

abstract class DefenderBotConfiguration {
    public String frontLeftMotorName = null;
    public String frontRightMotorName = null;
    public String rearLeftMotorName = null;
    public String rearRightMotorName = null;

    public Double forwardSecondsPerInch = null;
    public Double sidewaysSecondsPerInch = null;

}