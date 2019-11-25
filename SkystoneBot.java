package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * By subclassing the DefenderBot class, we can leave year-to-year functionality
 * (like driving) in one place so that all improvements are passed on for the next
 * year. Year-specific manipulation or functionality, though, can be put here
 * to keep it separate.
 */

public class SkystoneBot extends DefenderBot {

	public SmartDcMotor liftMotor = null;
	public SmartDcMotor tiltMotor = null;
	public SmartDcMotor extendMotor = null;
	public SmartDcMotor grabMotor = null;

	public Servo leftFoundationGrabber = null;
	public Servo rightFoundationGrabber = null;

	public SkystoneVisionService skystoneVisionService = null;
	private Thread skystoneVisionServiceThread = null;


	@Override
	public void init(HardwareMap ahwMap, DefenderBotConfiguration botConfig) {
		super.init(ahwMap, botConfig);

		leftFoundationGrabber = ahwMap.get(Servo.class, botConfiguration.leftFoundationGrabberServoName);
		leftFoundationGrabber.setDirection(botConfiguration.leftFoundationGrabberServoDirection);

		rightFoundationGrabber = ahwMap.get(Servo.class, botConfiguration.rightFoundationGrabberServoName);
		rightFoundationGrabber.setDirection(botConfiguration.rightFoundationGrabberServoDirection);

		liftMotor = new SmartDcMotor(hwMap, botConfiguration.liftMotorName, botConfiguration.liftMotorForwardDirection);
		liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		extendMotor = new SmartDcMotor(hwMap, botConfiguration.extendMotorName, botConfiguration.extendMotorForwardDirection);
		extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		tiltMotor = new SmartDcMotor(hwMap, botConfiguration.tiltMotorName, botConfiguration.tiltMotorForwardDirection);
		tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		grabMotor = new SmartDcMotor(hwMap, botConfiguration.grabMotorName, botConfiguration.grabMotorForwardDirection);
// 		grabMotor.setTargetPosition(0);
		grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    		grabMotor.setPower(botConfiguration.grabMotorPower);

// 		grabMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		stopManipulatorMotors();

	}

    //--------------------------------------------------------------------------------------------

    public void activateSkystoneVisionService() {
		skystoneVisionService = new SkystoneVisionService(250, botConfiguration);
		skystoneVisionServiceThread = new Thread(skystoneVisionService);
		skystoneVisionServiceThread.start();
    }

    //--------------------------------------------------------------------------------------------

    public void stopManipulatorMotors() {
	    liftMotor.setPower(0);
	    tiltMotor.setPower(0);
	    extendMotor.setPower(0);
// 	    grabMotor.setPower(0); // we don't actually want the grab/claw to have 0 power or it won't actually go to the right position.
	}

    //--------------------------------------------------------------------------------------------

	@Override
	public void stopAllMotors() {
		super.stopAllMotors();
		stopManipulatorMotors();
	}

    //--------------------------------------------------------------------------------------------

    	public void moveLiftUp(double power) {
	    	power = Math.abs(power);
	    	liftMotor.setDirectionForward();
	    	liftMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void moveLiftDown(double power) {
	    	power = Math.abs(power);
	    	liftMotor.setDirectionReverse();
	    	liftMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void stopLiftMotor() {
	    	liftMotor.setPower(0);
    	}

    //--------------------------------------------------------------------------------------------

    	public void tiltArmUp(double power) {
	    	power = Math.abs(power);
	    	tiltMotor.setDirectionForward();
	    	tiltMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void tiltArmDown(double power) {
	    	power = Math.abs(power);
	    	tiltMotor.setDirectionReverse();
	    	tiltMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void stopTiltMotor() {
	    	tiltMotor.setPower(0);
    	}


    //--------------------------------------------------------------------------------------------

    	public void extendArm(double power) {
	    	power = Math.abs(power);
	    	extendMotor.setDirectionForward();
	    	extendMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void retractArm(double power) {
	    	power = Math.abs(power);
	    	extendMotor.setDirectionReverse();
	    	extendMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void stopExtendMotor() {
	    	extendMotor.setPower(0);
    	}


    //--------------------------------------------------------------------------------------------

    	public void grabBlock(double power) {
/*
	    	power = Math.abs(power);
	    	grabMotor.setDirectionForward();
	    	grabMotor.setPower(power);
*/
    	}

    	public void captureBlock() {
	    	grabMotor.setTargetPosition(botConfiguration.capturePosition);
    	}

    	public void resetClaw() {
	    	grabMotor.setTargetPosition(botConfiguration.resetPosition);
    	}

    //--------------------------------------------------------------------------------------------

    	public void releaseBlock(double power) {
/*
	    	power = Math.abs(power);
	    	grabMotor.setDirectionReverse();
	    	grabMotor.setPower(power);
*/
    	}

    	public void releaseBlock() {
	    	grabMotor.setTargetPosition(botConfiguration.releasePosition);

    	}

    //--------------------------------------------------------------------------------------------

    	public void stopGrabMotor() {
	    	grabMotor.setPower(0);
    	}

    //--------------------------------------------------------------------------------------------

    public void grabFoundation() {
	    leftFoundationGrabber.setPosition(1.2);
	    rightFoundationGrabber.setPosition(1.2);
    }

    //--------------------------------------------------------------------------------------------

    public void releaseFoundation() {
	    leftFoundationGrabber.setPosition(0);
	    rightFoundationGrabber.setPosition(0);
    }

    //--------------------------------------------------------------------------------------------

	@Override
	public void shutdown() {
		super.shutdown();
		if (skystoneVisionService != null) {
			skystoneVisionService.stop();
		}

	}


}