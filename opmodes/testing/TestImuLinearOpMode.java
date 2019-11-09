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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Random;

/**
 * This op mode is currently used for testing functionality *
 */
@Autonomous(name="Testing: IMU", group="Linear Opmode")
@Disabled
public class TestImuLinearOpMode extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private DefenderBot productionBot = new DefenderBot();

    // This method is not perfect yet. Ideally should be broken out into a separate interface or a DefenderBot method.
    // Need to add ability to break on sensor input, ideally we pass allow an array of functions/methods to be passed
    // that are called every "tick"
    public void wait(double time) {
	    double start = runtime.milliseconds();
	    while (runtime.milliseconds() < (start + time)) {
		    sleep(500); // This "tick" value should perhaps be a default value but changeable in the wait call
	    }
    }

    @Override
    public void runOpMode() {
		productionBot.init(hardwareMap, new ProductionTestConfiguration());

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		productionBot.calibrateIMU();

		while (!isStopRequested() && !productionBot.imu.isGyroCalibrated()) {
			sleep(50);
			idle();
		}

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			double leftStickY = -gamepad1.left_stick_y;
			double leftStickX = gamepad1.left_stick_x;

			double rightStickY = -gamepad1.right_stick_y;
			double rightStickX = gamepad1.right_stick_x;


			if (gamepad1.dpad_up) {
				productionBot.driveBasedOnMovementSettings(productionBot.forwardMotion.scale(0.4));
				sleep(200);
				while (!productionBot.isNotMovingAccordingToImu3Axes()) {
					AngularVelocity omegas = productionBot.imu.getAngularVelocity();
					omegas.toAngleUnit(AngleUnit.DEGREES);
					telemetry.addData("angular v x:", String.format("%f", omegas.xRotationRate));
					telemetry.addData("angular v y:", String.format("%f", omegas.yRotationRate));
					telemetry.addData("angular v z:", String.format("%f", omegas.zRotationRate));
					telemetry.update();
					sleep(200);
				}
				productionBot.driveBasedOnMovementSettings(productionBot.reverseMotion.scale(0.6));
				sleep(1000);

			} else if (gamepad1.start) {
				productionBot.stopAllMotors();

			} else if (gamepad1.left_trigger > 0) {
				productionBot.driveLeft(gamepad1.left_trigger, gamepad1.left_trigger);

			} else if (gamepad1.right_trigger > 0) {
				productionBot.driveRight(gamepad1.right_trigger, gamepad1.right_trigger);


			} else if (leftStickY > 0) {

		   		// Forward and Right
				if (leftStickX > 0) {
					SimpleMovementSettings foo = SimpleMovementSettings.combine(productionBot.forwardMotion, productionBot.turnRightMotion, leftStickX, leftStickY);
	// 				telemetry.addData("movement", foo.toString());
					productionBot.driveBasedOnMovementSettings(foo);

				// Forward and Left
				} else if (leftStickX < 0) {
					SimpleMovementSettings foo = SimpleMovementSettings.combine(productionBot.forwardMotion, productionBot.turnLeftMotion, leftStickX, leftStickY);
	// 				telemetry.addData("movement", foo.toString());
					productionBot.driveBasedOnMovementSettings(foo);

				// Just Forward
				} else {
	// 				telemetry.addData("movement", productionBot.forwardMotion.scale(leftStickY).toString());
					productionBot.driveBasedOnMovementSettings(productionBot.forwardMotion.scale(leftStickY));
				}


			} else if (leftStickY < 0) {

				// Reverse and Right
				if (leftStickX > 0) {
					SimpleMovementSettings foo = SimpleMovementSettings.combine(productionBot.reverseMotion, productionBot.turnRightMotion, leftStickX, leftStickY);
	// 				telemetry.addData("movement", foo.toString());
					productionBot.driveBasedOnMovementSettings(foo);

				// Reverse and Left
				} else if (leftStickX < 0) {
					SimpleMovementSettings foo = SimpleMovementSettings.combine(productionBot.reverseMotion, productionBot.turnLeftMotion, leftStickX, leftStickY);
	// 				telemetry.addData("movement", foo.toString());
					productionBot.driveBasedOnMovementSettings(foo);

				// Just Reverse
				} else {
	// 				telemetry.addData("movement", productionBot.reverseMotion.scale(leftStickY).toString());
					productionBot.driveBasedOnMovementSettings(productionBot.reverseMotion.scale(leftStickY));
				}




			} else if (leftStickX > 0) {
				SimpleMovementSettings foo = productionBot.turnRightMotion.scale(leftStickX);
				telemetry.addData("movement", foo.toString());
				productionBot.turnClockwise(leftStickX, leftStickX);


			} else if (leftStickX < 0) {
				SimpleMovementSettings foo = productionBot.turnLeftMotion.scale(leftStickX);
				telemetry.addData("movement", foo.toString());
				productionBot.turnCounterClockwise(leftStickX, leftStickX);
			} else {
				productionBot.driveBasedOnMovementSettings(productionBot.stopMotion);
			}



			Orientation orientation = productionBot.imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
			telemetry.addData("X Axis", orientation.firstAngle);
			telemetry.addData("Y Axis", orientation.secondAngle);
			telemetry.addData("Z Axis", orientation.thirdAngle);

			AngularVelocity omegas = productionBot.imu.getAngularVelocity();
			omegas.toAngleUnit(AngleUnit.DEGREES);
			telemetry.addData("angular v x:", String.format("%f", omegas.xRotationRate));
			telemetry.addData("angular v y:", String.format("%f", omegas.yRotationRate));
			telemetry.addData("angular v z:", String.format("%f", omegas.zRotationRate));
			telemetry.addData("in motion", productionBot.isInMotionAccordingToImu() ? "yes" : "no");
			telemetry.update();

		}

/*
		productionBot.driveBasedOnMovementSettings(forwardMotion.scale(0.5));
		sleep(productionBot.millisecondsForDistanceForward(36));
*/


/*
	   while (opModeIsActive() && !productionBot.isFrontTouching()) {
		   productionBot.driveForward(0.5, 0.5);
	   }
	   productionBot.driveBackward(0.5,0.5);
*/


/*
        productionBot.driveForward(1,1);
       sleep(1500);
*/

/*
		Random rand = new Random();
		while (opModeIsActive()) {
		   if (productionBot.isFrontTouching()) {
			   productionBot.driveBackward(0.5,0.5);
			   sleep(500);
			   int r = rand.nextInt(10);
			   if (r > 7) {
				   productionBot.turnClockwise(0.5, 0.5);
				   sleep(1000); // this sleep time is enough to guarantee the bot turns 1/4
				} else if (r < 4) {
				   productionBot.turnCounterClockwise(0.5, 0.5);
				   sleep(1000); // this sleep time is enough to guarantee the bot turns 1/4
			   } else {
				   productionBot.turnClockwise(0.5, 0.5);
				   sleep(2000); // this sleep time is enough to guarantee the bot turns 1/2

			   }

		   }
		   productionBot.driveForward(0.5, 0.5);
	   }
*/
// 	   productionBot.driveBackward(0.5,0.5);

        // run until the end of the match (driver presses STOP)
/*    while (opModeIsActive()) {

                       // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Touching: ", productionBot.isFrontTouching() ? "yes" : "no");
           //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
        */
    }
}
