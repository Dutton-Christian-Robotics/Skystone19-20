package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class ProductionTestConfiguration extends SkystoneBotConfiguration {

	public ProductionTestConfiguration() {
		frontLeftMotorName 		= "FRONT LEFT MOTOR";
		frontLeftMotorType		= SmartDcMotor.MotorType.TORQUENADO;
		frontLeftMotorLocation	= DefenderBot.MotorLocation.LEFT;

		frontRightMotorName		= "FRONT RIGHT MOTOR";
		frontRightMotorType		= SmartDcMotor.MotorType.TORQUENADO;
		frontRightMotorLocation	= DefenderBot.MotorLocation.RIGHT;

		rearLeftMotorName		= "REAR LEFT MOTOR";
		rearLeftMotorType		= SmartDcMotor.MotorType.TORQUENADO;
		rearLeftMotorLocation	= DefenderBot.MotorLocation.LEFT;

		rearRightMotorName		= "REAR RIGHT MOTOR";
		rearRightMotorType		= SmartDcMotor.MotorType.TORQUENADO;
		rearRightMotorLocation	= DefenderBot.MotorLocation.RIGHT;


		frontLeftMotorForwardDirection = DcMotor.Direction.REVERSE;
		frontRightMotorForwardDirection = DcMotor.Direction.FORWARD;
		rearLeftMotorForwardDirection = DcMotor.Direction.REVERSE;
		rearRightMotorForwardDirection = DcMotor.Direction.FORWARD;


		// Names for manipulator motors
		liftMotorName = "LIFT MOTOR";
		extendMotorName = "EXTEND MOTOR";
		grabMotorName = "CLAW MOTOR";
		tiltMotorName = "TILT MOTOR";


		liftMotorForwardDirection = DcMotor.Direction.FORWARD;
		extendMotorForwardDirection = DcMotor.Direction.FORWARD;
		grabMotorForwardDirection = DcMotor.Direction.REVERSE;
		tiltMotorForwardDirection = DcMotor.Direction.FORWARD;

		leftFoundationGrabberServoName = "GRABBER LEFT";
		leftFoundationGrabberServoDirection = Servo.Direction.REVERSE;

		rightFoundationGrabberServoName = "GRABBER RIGHT";
		rightFoundationGrabberServoDirection = Servo.Direction.FORWARD;

		// Configuration for grab/claw open and close positions
		grabMotorPower = 1;
		capturePosition = 90;
		releasePosition = -20;
		openPosition = -50;
		neutralPosition = 0;


		tiltPositionForPreCapture = 100;
		extendPositionForPreCapture = 600;
		liftPositionForPreCapture = 0;

		tiltPositionForCapture = -150;
		extendPositionForCapture = 600;
		liftPositionForCapture = 0;

		tiltPositionForTravel = 50;
		extendPositionForTravel = 150;
		liftPositionForTravel = 0;

		tiltPositionForStart = 0;
		extendPositionForStart = 0;
		liftPositionForStart = 0;

		tiltPositionForPlacement = 100;
		extendPositionForPlacement = 350;
		liftPositionForPlacement = 0;



		forwardSecondsPerInch	= 0.05; //carpet at home
		sidewaysSecondsPerInch	= 0.0690476190476191;

		headingAxis = InternalGyroscopeService.Axis.Z;

		useExternalWebcam = true;
		externalWebcamName = "WEBCAM";

		webcam_dX = 7.0f * Measurements.mmPerInch;
		webcam_dY = 0.0f * Measurements.mmPerInch;
		webcam_dZ = 4.0f * Measurements.mmPerInch;

		webcam_rX = 90;
		webcam_rY = 0;
		webcam_rZ = 180;


		vuforiaKey = "AUIxotb/////AAABmUiA7l9qRE16hqtSNI3A9PcEZRKBFEpztpXDtfcAsQJuv2fNHk4EHnNOpnt+hQVkB2gqjKMH7lBzvwPDrU4sQdXPO3HyuqKkA0QLoYP/QozKWgYx79lKOipOWtCs6RQGO8zPIWF12SqUrSf/zccfT9P4cwafrgHcxP/zlNJJUmSNCd3tcZxYKKOfvK2g97BmM2oad142iXfrvZPKIm96Re+krEVYy5cMb8pORKYzjFCrqfbFP8JarICUpI1plvrl5DwgWJGrwZLle/AEDuaCOSv5wWrbqHW7gq8SR9VMIG79eKZRXI1P6gEPMsxRiUchKwBwVeDXothphPoWtn1dNPLjg4lp2b4Xmok4bITUF+aA";

	}
}