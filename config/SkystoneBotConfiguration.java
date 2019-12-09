/*
	We decided that if we're separating base/every-year DefenderBot functionality from specific challenge
	functionality into base and child classes, maybe we should do the same for configuration.
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

abstract class SkystoneBotConfiguration extends DefenderBotConfiguration {


	public String liftMotorName = null;
	public DcMotor.Direction liftMotorForwardDirection = null;

	public String tiltMotorName = null;
	public DcMotor.Direction tiltMotorForwardDirection = null;

	public String extendMotorName = null;
	public DcMotor.Direction extendMotorForwardDirection = null;

	public String grabMotorName = null;
	public DcMotor.Direction grabMotorForwardDirection = null;
	public int grabMotorPower = 0;
	public int capturePosition = 0;
	public int releasePosition = 0;
	public int openPosition = 0;
	public int neutralPosition = 0;

	public int tiltPositionForCapture = 0;
	public int extendPositionForCapture = 0;
	public int liftPositionForCapture = 0;

	public int tiltPositionForTravel = 0;
	public int extendPositionForTravel = 0;
	public int liftPositionForTravel = 0;

	public int tiltPositionForStart = 0;
	public int extendPositionForStart = 0;
	public int liftPositionForStart = 0;

	public int tiltPositionForStart = 0;
	public int extendPositionForStart = 0;
	public int liftPositionForStart = 0;

	public int tiltPositionForPlacement = 0;
	public int extendPositionForPlacement = 0;
	public int liftPositionForPlacement = 0;


	public String leftFoundationGrabberServoName = null;
	public Servo.Direction leftFoundationGrabberServoDirection = null;

	public String rightFoundationGrabberServoName = null;
	public Servo.Direction rightFoundationGrabberServoDirection = null;


}