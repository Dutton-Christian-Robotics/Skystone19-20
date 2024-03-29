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


	public void init(HardwareMap ahwMap, SkystoneBotConfiguration botConfig) {
		super.init(ahwMap, botConfig);

		leftFoundationGrabber = ahwMap.get(Servo.class, botConfig.leftFoundationGrabberServoName);
		leftFoundationGrabber.setDirection(botConfig.leftFoundationGrabberServoDirection);

		rightFoundationGrabber = ahwMap.get(Servo.class, botConfig.rightFoundationGrabberServoName);
		rightFoundationGrabber.setDirection(botConfig.rightFoundationGrabberServoDirection);

		liftMotor = new SmartDcMotor(hwMap, botConfig.liftMotorName, botConfig.liftMotorForwardDirection);
		liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 		liftMotor.setTargetPosition(0);
		liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		extendMotor = new SmartDcMotor(hwMap, botConfig.extendMotorName, botConfig.extendMotorForwardDirection);
		extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 		extendMotor.setTargetPosition(0);
		extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		tiltMotor = new SmartDcMotor(hwMap, botConfig.tiltMotorName, botConfig.tiltMotorForwardDirection);
		tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// 		tiltMotor.setTargetPosition(0);
		tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		grabMotor = new SmartDcMotor(hwMap, botConfig.grabMotorName, botConfig.grabMotorForwardDirection);
		grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		grabMotor.setTargetPosition(0);
		grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    		grabMotor.setPower(botConfig.grabMotorPower);

// 		grabMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		stopManipulatorMotors();

	}

    // --------------------------------------------------------------------------------------------

    public void activateSkystoneVisionService(int cmvid) {
		skystoneVisionService = new SkystoneVisionService(250, botConfiguration, hwMap, cmvid);
		skystoneVisionServiceThread = new Thread(skystoneVisionService);
		skystoneVisionServiceThread.start();
    }

    public void activateSkystoneVisionService() {
	    activateSkystoneVisionService(-1);
    }

    // --------------------------------------------------------------------------------------------

    public void stopManipulatorMotors() {
	    liftMotor.setPower(0);
	    tiltMotor.setPower(0);
	    extendMotor.setPower(0);
// 	    grabMotor.setPower(0); // we don't actually want the grab/claw to have 0 power or it won't actually go to the right position.
	}

    // --------------------------------------------------------------------------------------------

	@Override
	public void stopAllMotors() {
		super.stopAllMotors();
		stopManipulatorMotors();
	}


    // --------------------------------------------------------------------------------------------

    public void manipulatorsToPreCapturePosition() {
		openClaw();
		setTiltPosition(botConfiguration.tiltPositionForPreCapture);
		setExtendPosition(botConfiguration.extendPositionForPreCapture);
// 		setLiftPosition(botConfiguration.liftPositionForCapture);
    }

    // --------------------------------------------------------------------------------------------

    public void manipulatorsToCapturePosition() {
		openClaw();
		setExtendPosition(botConfiguration.extendPositionForCapture);
		setTiltPosition(botConfiguration.tiltPositionForCapture);
// 		setLiftPosition(botConfiguration.liftPositionForCapture);
    }

    // --------------------------------------------------------------------------------------------

    public void manipulatorsToStartPosition() {
		setExtendPosition(botConfiguration.extendPositionForStart);
		setTiltPosition(botConfiguration.tiltPositionForStart);
// 		setLiftPosition(botConfiguration.liftPositionForStart);
		neutralClaw();
    }

    // --------------------------------------------------------------------------------------------

    public void manipulatorsToTravelPosition() {
		captureBlock();
		setExtendPosition(botConfiguration.extendPositionForTravel);
		setTiltPosition(botConfiguration.tiltPositionForTravel);
// 		setLiftPosition(botConfiguration.liftPositionForTravel);
    }

    // --------------------------------------------------------------------------------------------

    public void manipulatorsToPlacementPosition() {
		captureBlock();
		setExtendPosition(botConfiguration.extendPositionForPlacement);
		setTiltPosition(botConfiguration.tiltPositionForPlacement);
// 		setLiftPosition(botConfiguration.liftPositionForPlacement);
    }

    // --------------------------------------------------------------------------------------------

    	public void moveLiftUp(double power) {
	    	power = Math.abs(power);
	    	liftMotor.setDirectionForward();
	    	liftMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void moveLiftDown(double power) {
	    	power = Math.abs(power);
	    	liftMotor.setDirectionReverse();
	    	liftMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void stopLiftMotor() {
	    	liftMotor.setPower(0);
    	}

	public void setLiftPosition(int i) {
		setTiltPosition(i, 1);
	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

	public void setLiftPosition(int i, double power) {
	    	liftMotor.setPower(power);
		liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor.setTargetPosition(i);
		while(liftMotor.isBusy()) {
		}
		liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    	liftMotor.setPower(0);
    }


    // --------------------------------------------------------------------------------------------

    	public void tiltArmUp(double power) {
	    	power = Math.abs(power);
	    	tiltMotor.setDirectionForward();
	    	tiltMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void tiltArmDown(double power) {
	    	power = Math.abs(power);
	    	tiltMotor.setDirectionReverse();
	    	tiltMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void stopTiltMotor() {
	    	tiltMotor.setPower(0);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

	public void setTiltPosition(int i) {
		setTiltPosition(i, 1);
	}

	public void setTiltPosition(int i, double power) {
	    	tiltMotor.setPower(power);
		tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		tiltMotor.setTargetPosition(i);
		while(tiltMotor.isBusy()) {
		}
		tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    	tiltMotor.setPower(0);
    }


    // --------------------------------------------------------------------------------------------

    	public void extendArm(double power) {
	    	power = Math.abs(power);
	    	extendMotor.setDirectionForward();
	    	extendMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void retractArm(double power) {
	    	power = Math.abs(power);
	    	extendMotor.setDirectionReverse();
	    	extendMotor.setPower(power);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void stopExtendMotor() {
	    	extendMotor.setPower(0);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    public void setExtendPosition(int i) {
	    	extendMotor.setPower(1);
		extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		extendMotor.setTargetPosition(i);
		while(extendMotor.isBusy()) {
		}
		extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	    	extendMotor.setPower(0);
    }
    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -


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

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void neutralClaw() {
	    	grabMotor.setTargetPosition(botConfiguration.neutralPosition);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void openClaw() {
	    	grabMotor.setTargetPosition(botConfiguration.openPosition);
    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void releaseBlock() {
	    	grabMotor.setTargetPosition(botConfiguration.releasePosition);

    	}

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void releaseBlock(double power) {
/*
	    	power = Math.abs(power);
	    	grabMotor.setDirectionReverse();
	    	grabMotor.setPower(power);
*/
    	}


    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

    	public void stopGrabMotor() {
	    	grabMotor.setPower(0);
    	}

    // --------------------------------------------------------------------------------------------

    public void grabFoundation() {
	    leftFoundationGrabber.setPosition(0.3);
	    rightFoundationGrabber.setPosition(0.3);
    }

    // --------------------------------------------------------------------------------------------

    public void releaseFoundation() {
	    leftFoundationGrabber.setPosition(0);
	    rightFoundationGrabber.setPosition(0);
    }

    // --------------------------------------------------------------------------------------------

	@Override
	public void shutdown() {
		super.shutdown();
		if (skystoneVisionService != null) {
			skystoneVisionService.stop();
		}

	}


}