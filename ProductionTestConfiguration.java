package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

class ProductionTestConfiguration extends DefenderBotConfiguration {

	public ProductionTestConfiguration() {
		frontLeftMotorName 		= "FRONT LEFT MOTOR";
		frontRightMotorName		= "FRONT RIGHT MOTOR";
		rearLeftMotorName		= "REAR LEFT MOTOR";
		rearRightMotorName		= "REAR RIGHT MOTOR";


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
		grabMotorName = "GRAB MOTOR";
		tiltMotorName = "TILT MOTOR";

		liftMotorForwardDirection = DcMotor.Direction.REVERSE;
		extendMotorForwardDirection = DcMotor.Direction.FORWARD;
		grabMotorForwardDirection = DcMotor.Direction.REVERSE;
		tiltMotorForwardDirection = DcMotor.Direction.FORWARD;

		forwardSecondsPerInch	= 0.0571895424836602;
		sidewaysSecondsPerInch	= 0.0690476190476191;
	}
}