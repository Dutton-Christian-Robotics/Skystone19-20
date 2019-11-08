/*
	Speaking from experience, here: in the heat of competition it's easy to start rewriting code to make
	last minute changes. To prevent "tweaking" of code that creates inadvertant problems, and to make
	things more flexible, we use a special configuration class rather than hardcoding values into
	the actual working code of the bot.
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

abstract class DefenderBotConfiguration {
	public String frontLeftMotorName = null;
	public SmartDcMotor.MotorType frontLeftMotorType = null;
	public DefenderBot.MotorLocation frontLeftMotorLocation = null;
	public DcMotor.Direction frontLeftMotorForwardDirection = null;

	public String frontRightMotorName = null;
	public SmartDcMotor.MotorType frontRightMotorType = null;
	public DefenderBot.MotorLocation frontRightMotorLocation = null;
	public DcMotor.Direction frontRightMotorForwardDirection = null;

	public String rearLeftMotorName = null;
	public SmartDcMotor.MotorType rearLeftMotorType = null;
	public DefenderBot.MotorLocation rearLeftMotorLocation = null;
	public DcMotor.Direction rearLeftMotorForwardDirection = null;

	public String rearRightMotorName = null;
	public SmartDcMotor.MotorType rearRightMotorType = null;
	public DefenderBot.MotorLocation rearRightMotorLocation = null;
	public DcMotor.Direction rearRightMotorForwardDirection = null;


	public String liftMotorName = null;
	public DcMotor.Direction liftMotorForwardDirection = null;

	public String tiltMotorName = null;
	public DcMotor.Direction tiltMotorForwardDirection = null;

	public String extendMotorName = null;
	public DcMotor.Direction extendMotorForwardDirection = null;

	public String grabMotorName = null;
	public DcMotor.Direction grabMotorForwardDirection = null;

	public Double forwardSecondsPerInch = null;
	public Double sidewaysSecondsPerInch = null;

	public InternalGyroscopeService.Axis headingAxis = null;

	/*
		Not used currently, this is a convenience method that could let us setup our motors with less duplication of code,
		such as by iterating through an array of motor names.
	*/
/*
	public DcMotor.Direction forwardDirectionForMotor(String motorName) {
		try {
			return this.getClass().getField(motorName + "ForwardDirection").get(this));
		} catch (Exception e) {
			return DcMotor.Direction.FORWARD;
		}
	}
*/


}