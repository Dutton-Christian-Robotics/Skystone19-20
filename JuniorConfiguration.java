package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

class JuniorConfiguration extends DefenderBotConfiguration {

	public JuniorConfiguration() {
		frontLeftMotorName 		= "Front Left Orange";
		frontRightMotorName		= "Front Right Red";
		rearLeftMotorName		= "Rear Left Camo";
		rearRightMotorName		= "Rear Right Green";


		frontLeftMotorForwardDirection = DcMotor.Direction.FORWARD;
		frontRightMotorForwardDirection = DcMotor.Direction.FORWARD;
		rearLeftMotorForwardDirection = DcMotor.Direction.FORWARD;
		rearRightMotorForwardDirection = DcMotor.Direction.FORWARD;


/*
		liftMotorName = null;
		extendMotorName = null;
		grabMotorName = null;
		tiltMotorName = null;

		liftMotorForwardDirection = null;
		extendMotorForwardDirection = null;
		grabMotorForwardDirection = null;
		tiltMotorForwardDirection = null;
*/

		forwardSecondsPerInch	= 0.0571895424836602;
		sidewaysSecondsPerInch	= 0.0690476190476191;
	}
}