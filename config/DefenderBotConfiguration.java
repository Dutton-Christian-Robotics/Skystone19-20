/*
	Speaking from experience, here: in the heat of competition it's easy to start rewriting code to make
	last minute changes. To prevent "tweaking" of code that creates inadvertant problems, and to make
	things more flexible, we use a special configuration class rather than hardcoding values into
	the actual working code of the bot.
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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




	public Double forwardSecondsPerInch = null;
	public Double sidewaysSecondsPerInch = null;

	public InternalGyroscopeService.Axis headingAxis = null;

	public String vuforiaKey = null;
	public String externalWebcamName = null;
	public boolean useExternalWebcam = false;

	public VuforiaLocalizer.CameraDirection cameraChoice = VuforiaLocalizer.CameraDirection.BACK;
	public boolean phoneIsPortrait = false;

	public float webcam_dX = 0;
	public float webcam_dY = 0;
	public float webcam_dZ = 0;

	public float webcam_rX = 0;
	public float webcam_rY = 0;
	public float webcam_rZ = 0;


}