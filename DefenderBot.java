/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class DefenderBot
{
    public SmartDcMotor frontLeftDrive = null;
    public SmartDcMotor frontRightDrive = null;
    public SmartDcMotor rearLeftDrive = null;
    public SmartDcMotor rearRightDrive = null;

    public SmartDcMotor liftMotor = null;
    public SmartDcMotor tiltMotor = null;
    public SmartDcMotor extendMotor = null;
    public SmartDcMotor grabMotor = null;

//     public TouchSensor frontTouch = null;

    private HardwareMap hwMap           = null;
    private DefenderBotConfiguration botConfiguration 	= null;
    private ElapsedTime period  = new ElapsedTime();

    public Double forwardSecondsPerInch = null;
    public Double sidewaysSecondsPerInch = null;

    public DefenderBot() {

    }

    public void init(HardwareMap ahwMap, DefenderBotConfiguration botConfig) {
		// Save reference to Hardware map and config file
		hwMap = ahwMap;
		botConfiguration = botConfig;

		// Define and Initialize Motors and timing using config file
		// Using the SmartDcMotor class allows us to separate out the CONCEPTUAL direction we want
		// the motor to turn from the PHYSICAL direction of how the motor is mounted.
		frontLeftDrive = new SmartDcMotor(hwMap, botConfiguration.frontLeftMotorName, botConfiguration.frontLeftMotorForwardDirection);
		frontRightDrive = new SmartDcMotor(hwMap, botConfiguration.frontRightMotorName, botConfiguration.frontRightMotorForwardDirection);
		rearLeftDrive = new SmartDcMotor(hwMap, botConfiguration.rearLeftMotorName, botConfiguration.rearLeftMotorForwardDirection);
		rearRightDrive = new SmartDcMotor(hwMap, botConfiguration.rearRightMotorName, botConfiguration.rearRightMotorForwardDirection);

		frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




		liftMotor = new SmartDcMotor(hwMap, botConfiguration.liftMotorName, botConfiguration.liftMotorForwardDirection);

		extendMotor = new SmartDcMotor(hwMap, botConfiguration.extendMotorName, botConfiguration.extendMotorForwardDirection);

		grabMotor = new SmartDcMotor(hwMap, botConfiguration.grabMotorName, botConfiguration.grabMotorForwardDirection);

		forwardSecondsPerInch = botConfiguration.forwardSecondsPerInch;
		sidewaysSecondsPerInch = botConfiguration.sidewaysSecondsPerInch;



// 		frontTouch = hwMap.get(TouchSensor.class, "front touch");


		// Set all motors to zero power
		stopAllMotors();

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //--------------------------------------------------------------------------------------------

    public void stopDriveMotors() {
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		rearLeftDrive.setPower(0);
		rearRightDrive.setPower(0);
    }

    //--------------------------------------------------------------------------------------------

    public void stopManipulatorMotors() {
	    liftMotor.setPower(0);
// 	    tiltMotor.setPower(0);
	    extendMotor.setPower(0);
	    grabMotor.setPower(0);
    }

    //--------------------------------------------------------------------------------------------

    public void stopAllMotors() {
		stopDriveMotors();
		stopManipulatorMotors();
    }

    //--------------------------------------------------------------------------------------------

/*
	THE FOLLOWING CODE DRIVE FUNCTIONS SHOULD NOT BE CONSIDERED FINAL YET.

	I'm trying to use method overloading so that we have a variety of wys to call drive functions. There are a couple of problems
	in Java I'm trying to get around, namely that you can't provide default method argument values and simple types like
	double can't be compared against null (double will be set to 0 if not provided, and since 0 is a meaningful value for
	motor power, it's not an ideal value).

	We'd like drive functions to be flexible. Able to be called simply (perhaps with just one power that's applied to all wheels).
	But also with the ability to set power individually for all wheels, once we need to tweak final performance.
*/

	public void setDrivePower(Double frontLeftPower, Double frontRightPower) {
		setDrivePower(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
	}
	public void setDrivePower(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {
		if (rearLeftPower == null) {
		    rearLeftPower = frontLeftPower;
		}
		if (rearRightPower == null) {
		    rearRightPower = frontRightPower;
		}
		frontLeftDrive.setPower(frontLeftPower);
		frontRightDrive.setPower(frontRightPower);
		rearLeftDrive.setPower(rearLeftPower);
		rearRightDrive.setPower(rearRightPower);
    }

    //--------------------------------------------------------------------------------------------

	public void driveForward(double frontLeftPower, double frontRightPower) {
		driveForward(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
	}
	public void driveForward(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
		driveForward(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}
	public void driveForward(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

		frontLeftDrive.setDirectionForward();
		frontRightDrive.setDirectionForward();
		rearLeftDrive.setDirectionForward();
		rearRightDrive.setDirectionForward();

		setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    //--------------------------------------------------------------------------------------------


    public void driveBackward(double frontLeftPower, double frontRightPower) {
	    driveBackward(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
    }
    public void driveBackward(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
	    driveBackward(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}
    public void driveBackward(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

        frontLeftDrive.setDirectionReverse();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionReverse();
        rearRightDrive.setDirectionReverse();

	   setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    //--------------------------------------------------------------------------------------------

    public void stopDriving() {
	   stopDriveMotors();
    }

    //--------------------------------------------------------------------------------------------

    public void driveLeft(double frontLeftPower, double frontRightPower) {
	    driveLeft(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
    }
    public void driveLeft(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
	    driveLeft(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}
    public void driveLeft(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

		/*
			Once we started programming the "real" bot we had a strange problem. After capturing the "physical"
			forward direction for each wheel, we could get it to drive forward or backward fine. But when going
			sideways or diagonal, the right-side wheels turned the wrong direction. Still trying to figure out why.
			Until then, we bandaid the problem by switching which way we say we want to go.
		*/
        frontLeftDrive.setDirectionReverse();
//         frontRightDrive.setDirectionForward();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionForward();
//         rearRightDrive.setDirectionReverse();
        rearRightDrive.setDirectionForward();

	   setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    //--------------------------------------------------------------------------------------------

    public void driveRight(double frontLeftPower, double frontRightPower) {
	    driveRight(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
    }
    public void driveRight(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
	    driveRight(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}

	public void driveRight(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

		/*
			Once we started programming the "real" bot we had a strange problem. After capturing the "physical"
			forward direction for each wheel, we could get it to drive forward or backward fine. But when going
			sideways or diagonal, the right-side wheels turned the wrong direction. Still trying to figure out why.
			Until then, we bandaid the problem by switching which way we say we want to go.
		*/
		frontLeftDrive.setDirectionForward();
//         frontRightDrive.setDirectionReverse();
		frontRightDrive.setDirectionForward();
		rearLeftDrive.setDirectionReverse();
//         rearRightDrive.setDirectionForward();
		rearRightDrive.setDirectionReverse();

		setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    //--------------------------------------------------------------------------------------------


    public void turnClockwise(double frontLeftPower, double frontRightPower) {
	    turnClockwise(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
    }
    public void turnClockwise(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
	    turnClockwise(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}

    public void turnClockwise(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

        frontLeftDrive.setDirectionForward();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionForward();
        rearRightDrive.setDirectionReverse();

	   setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);

    }

    //--------------------------------------------------------------------------------------------


    public void turnCounterClockwise(double frontLeftPower, double frontRightPower) {
	    turnCounterClockwise(frontLeftPower, frontRightPower, frontLeftPower, frontRightPower);
    }
    public void turnCounterClockwise(double frontLeftPower, double frontRightPower, double rearLeftPower, double rearRightPower) {
	    turnCounterClockwise(new Double(frontLeftPower), new Double(frontRightPower), new Double(rearLeftPower), new Double(rearRightPower));
	}

    public void turnCounterClockwise(Double frontLeftPower, Double frontRightPower, Double rearLeftPower, Double rearRightPower) {

        frontLeftDrive.setDirectionReverse();
        frontRightDrive.setDirectionForward();
        rearLeftDrive.setDirectionReverse();
        rearRightDrive.setDirectionForward();

	   setDrivePower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);

    }

    //--------------------------------------------------------------------------------------------


    public void turnClockwiseRearRight(double frontLeftPower) {
	    turnClockwiseRearRight(new Double(frontLeftPower));
	}
    public void turnClockwiseRearRight(Double leftPower) {

        frontLeftDrive.setDirectionForward();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionForward();
        rearRightDrive.setDirectionReverse();

	   setDrivePower(leftPower, new Double(0), leftPower, new Double(0));

    }

    //--------------------------------------------------------------------------------------------

    public void turnClockwiseRearAxis(double leftPower, double rightPower) {
	    turnClockwiseRearAxis(new Double(leftPower), new Double(rightPower));
	}
    public void turnClockwiseRearAxis(Double leftPower, Double rightPower) {

        frontLeftDrive.setDirectionForward();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionForward();
        rearRightDrive.setDirectionReverse();

	   setDrivePower(leftPower, rightPower, new Double(0), new Double(0));

    }

    //--------------------------------------------------------------------------------------------

/*
	The following drive functions have not been abstracted yet.
*/

    public void driveDiagonalForwardRight(double leftPower, double rightPower) {

        frontLeftDrive.setDirectionForward();
        frontRightDrive.setDirectionForward();
        rearLeftDrive.setDirectionForward();
        rearRightDrive.setDirectionForward();

        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(rightPower);

    }

    public void driveDiagonalForwardLeft(double leftPower, double rightPower) {

        frontLeftDrive.setDirectionForward();
        frontRightDrive.setDirectionForward();
        rearLeftDrive.setDirectionForward();
        rearRightDrive.setDirectionForward();

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(rightPower);
        rearLeftDrive.setPower(leftPower);
        rearRightDrive.setPower(0);

    }

    public void driveDiagonalBackwardLeft(double leftPower, double rightPower) {

        frontLeftDrive.setDirectionReverse();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionReverse();
        rearRightDrive.setDirectionReverse();

        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(rightPower);

    }

    public void driveDiagonalBackwardRight(double leftPower, double rightPower) {

        frontLeftDrive.setDirectionReverse();
        frontRightDrive.setDirectionReverse();
        rearLeftDrive.setDirectionReverse();
        rearRightDrive.setDirectionReverse();

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(rightPower);
        rearLeftDrive.setPower(leftPower);
        rearRightDrive.setPower(0);

    }

    //--------------------------------------------------------------------------------------------

    	public void moveLiftUp(double power) {
	    	liftMotor.setDirectionForward();
	    	liftMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void moveLiftDown(double power) {
	    	liftMotor.setDirectionReverse();
	    	liftMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void tiltArmUp(double power) {
	    	tiltMotor.setDirectionForward();
	    	tiltMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void tiltArmDown(double power) {
	    	tiltMotor.setDirectionReverse();
	    	tiltMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void extendArm(double power) {
	    	extendMotor.setDirectionForward();
	    	extendMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void retractArm(double power) {
	    	extendMotor.setDirectionReverse();
	    	extendMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void grabBlock(double power) {
	    	grabMotor.setDirectionForward();
	    	grabMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------

    	public void releaseBlock(double power) {
	    	grabMotor.setDirectionReverse();
	    	grabMotor.setPower(power);
    	}

    //--------------------------------------------------------------------------------------------


    public boolean isFrontTouching() {
	    return false;
// 	    return frontTouch.isPressed();
    }

 }

// ====================================================================================================

