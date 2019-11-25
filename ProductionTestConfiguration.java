package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

class ProductionTestConfiguration extends DefenderBotConfiguration {

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


		/*
			Once we started programming the "real" bot we had a strange problem. After capturing the "physical"
			forward direction for each wheel, we could get it to drive forward or backward fine. But when going
			sideways or diagonal, the right-side wheels turned the wrong direction. Still trying to figure out why.
		*/
		frontLeftMotorForwardDirection = DcMotor.Direction.REVERSE;
		frontRightMotorForwardDirection = DcMotor.Direction.FORWARD;
		rearLeftMotorForwardDirection = DcMotor.Direction.REVERSE;
		rearRightMotorForwardDirection = DcMotor.Direction.FORWARD;


		liftMotorName = "LIFT MOTOR";
		extendMotorName = "EXTEND MOTOR";
		grabMotorName = "CLAW MOTOR";
		tiltMotorName = "TILT MOTOR";

		grabMotorPower = 1;
		capturePosition = 0;
		releasePosition = 230;
		resetPosition = 90;


		liftMotorForwardDirection = DcMotor.Direction.REVERSE;
		extendMotorForwardDirection = DcMotor.Direction.FORWARD;
		grabMotorForwardDirection = DcMotor.Direction.REVERSE;
		tiltMotorForwardDirection = DcMotor.Direction.FORWARD;

		leftFoundationGrabberServoName = "GRABBER LEFT";
		leftFoundationGrabberServoDirection = Servo.Direction.REVERSE;

		rightFoundationGrabberServoName = "GRABBER RIGHT";
		rightFoundationGrabberServoDirection = Servo.Direction.FORWARD;



		forwardSecondsPerInch	= 0.05; //carpet at home
		sidewaysSecondsPerInch	= 0.0690476190476191;

		headingAxis = InternalGyroscopeService.Axis.Z;

		vuforiaKey = "AUIxotb/////AAABmUiA7l9qRE16hqtSNI3A9PcEZRKBFEpztpXDtfcAsQJuv2fNHk4EHnNOpnt+hQVkB2gqjKMH7lBzvwPDrU4sQdXPO3HyuqKkA0QLoYP/QozKWgYx79lKOipOWtCs6RQGO8zPIWF12SqUrSf/zccfT9P4cwafrgHcxP/zlNJJUmSNCd3tcZxYKKOfvK2g97BmM2oad142iXfrvZPKIm96Re+krEVYy5cMb8pORKYzjFCrqfbFP8JarICUpI1plvrl5DwgWJGrwZLle/AEDuaCOSv5wWrbqHW7gq8SR9VMIG79eKZRXI1P6gEPMsxRiUchKwBwVeDXothphPoWtn1dNPLjg4lp2b4Xmok4bITUF+aA";

	}
}