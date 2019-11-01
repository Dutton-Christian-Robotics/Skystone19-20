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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.Callable;
import java.util.*;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Beta: 2 Gamepad Driving", group="Iterative Opmode")

public class TwoGamepadTeleOpMode extends OpMode {
	private ElapsedTime runtime = new ElapsedTime();
	private DefenderBot bot = new DefenderBot();
	private double driveScaleFactor = 1;
	private double manipulatorScaleFactor = 1;
	private Debouncer decreaseDriveScaleFactorDebouncer;
	private Debouncer decreaseManipulatorScaleFactorDebouncer;

    /*
     * Code to run ONCE when the driver hits INIT
     */
	@Override
	public void init() {
		bot.init(hardwareMap, new ProductionTestConfiguration());

		// Tell the driver that initialization is complete.
		telemetry.addData("Status", "Initialized");
	}

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
	@Override
	public void init_loop() {
	}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
	@Override
	public void start() {
		runtime.reset();
		decreaseDriveScaleFactorDebouncer = new Debouncer(400, new Callable<Void>() {
			@Override
			public Void call() {
				driveScaleFactor -= 0.25;
				return null;
			}
		});
		decreaseManipulatorScaleFactorDebouncer = new Debouncer(400, new Callable<Void>() {
			@Override
			public Void call() {
				manipulatorScaleFactor -= 0.25;
				return null;
			}
		});
	}

	/*
      * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
      */
    @Override
    public void loop() {
		double gp1_leftStickY = -gamepad1.left_stick_y;
		double gp1_leftStickX = gamepad1.left_stick_x;
		double gp1_rightStickY = -gamepad1.right_stick_y;
		double gp1_rightStickX = gamepad1.right_stick_x;

		double gp2_leftStickY = -gamepad2.left_stick_y;
		double gp2_leftStickX = gamepad2.left_stick_x;
		double gp2_rightStickY = -gamepad2.right_stick_y;
		double gp2_rightStickX = gamepad2.right_stick_x;

		if (gamepad1.left_bumper) {
			try {
				// testing shows that the loop runs so fast that a single press and release of the button is read multiple times.
				// I'm using a Debouncer object to throttle how often it can be called.
				decreaseDriveScaleFactorDebouncer.run();
			} catch (Exception e) {

			}
			if (driveScaleFactor < 0.25) {
				driveScaleFactor = 0.25;
			}
		} else if (gamepad1.right_bumper) {
			driveScaleFactor = 1;
		}
		if (gamepad2.left_bumper) {
			try {
				// testing shows that the loop runs so fast that a single press and release of the button is read multiple times.
				// I'm using a Debouncer object to throttle how often it can be called.
				decreaseManipulatorScaleFactorDebouncer.run();
			} catch (Exception e) {

			}
			if (manipulatorScaleFactor < 0.25) {
				manipulatorScaleFactor = 0.25;
			}
		} else if (gamepad2.right_bumper) {
			manipulatorScaleFactor = 1;
		}

		// Emergency stop. Before all other conditions to ensure it takes precedent.
		if (gamepad1.start || gamepad2.start) {
			bot.stopAllMotors();

		// Drive sideways to the left
		} else if (gamepad1.left_trigger > 0) {
			bot.driveLeft(gamepad1.left_trigger * driveScaleFactor, gamepad1.left_trigger * driveScaleFactor);

		// Drive sideways to the right
		} else if (gamepad1.right_trigger > 0) {
			bot.driveRight(gamepad1.right_trigger * driveScaleFactor, gamepad1.right_trigger * driveScaleFactor);

		// Drive diagonal forward left
		} else if (gamepad1.dpad_up && gamepad1.dpad_left) {
			bot.driveBasedOnMovementSettings(bot.diagonalForwardLeftMotion.scale(driveScaleFactor));

		// Drive diagonal forward right
		} else if (gamepad1.dpad_up && gamepad1.dpad_right) {
			bot.driveBasedOnMovementSettings(bot.diagonalForwardRightMotion.scale(driveScaleFactor));

		// Drive diagonal reverse left
		} else if (gamepad1.dpad_down && gamepad1.dpad_left) {
			bot.driveBasedOnMovementSettings(bot.diagonalReverseLeftMotion.scale(driveScaleFactor));

		// Drive diagonal reverse right
		} else if (gamepad1.dpad_down && gamepad1.dpad_right) {
			bot.driveBasedOnMovementSettings(bot.diagonalReverseRightMotion.scale(driveScaleFactor));


		// Some movement forward
		} else if (gp1_leftStickY > 0) {

	   		// Forward and Right
			if (gp1_leftStickX > 0) {
				bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(bot.forwardMotion, bot.turnRightMotion, gp1_leftStickX, gp1_leftStickY).scale(driveScaleFactor));

			// Forward and Left
			} else if (gp1_leftStickX < 0) {
				bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(bot.forwardMotion, bot.turnLeftMotion, gp1_leftStickX, gp1_leftStickY).scale(driveScaleFactor));

			// Just Forward
			} else {
				bot.driveBasedOnMovementSettings(bot.forwardMotion.scale(gp1_leftStickY * driveScaleFactor));
			}


		// Some movement in reverse
		} else if (gp1_leftStickY < 0) {

			// Reverse and Right
			if (gp1_leftStickX > 0) {
				bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(bot.reverseMotion, bot.turnRightMotion, gp1_leftStickX, gp1_leftStickY).scale(driveScaleFactor));

			// Reverse and Left
			} else if (gp1_leftStickX < 0) {
				bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(bot.reverseMotion, bot.turnLeftMotion, gp1_leftStickX, gp1_leftStickY).scale(driveScaleFactor));

			// Just Reverse
			} else {
				bot.driveBasedOnMovementSettings(bot.reverseMotion.scale(gp1_leftStickY * driveScaleFactor));
			}

		// Pure turn to the right
		} else if (gp1_leftStickX > 0) {
			bot.driveBasedOnMovementSettings(bot.turnRightMotion.scale(gp1_leftStickX * driveScaleFactor));

		// Pure turn to the left
		} else if (gp1_leftStickX < 0) {
			bot.driveBasedOnMovementSettings(bot.turnLeftMotion.scale(gp1_leftStickX * driveScaleFactor));

		// Stop drive wheels
		} else {
			bot.driveBasedOnMovementSettings(bot.stopMotion);
		}


		// Tilt manipulator arm up and down with gamepad 2 right stick Y
		if (gp2_rightStickY > 0) {
			bot.tiltArmUp(gp2_rightStickY * manipulatorScaleFactor);
		} else if (gp2_rightStickY < 0) {
			bot.tiltArmDown(gp2_rightStickY * manipulatorScaleFactor);
		} else {
			bot.stopTiltMotor();
		}

		// Raise manipulator arm up and down with gamepad 2 left stick Y
		if (gp2_leftStickY > 0) {
			bot.moveLiftUp(gp2_leftStickY * manipulatorScaleFactor);
		} else if (gp2_leftStickY < 0) {
			bot.moveLiftDown(gp2_leftStickY * manipulatorScaleFactor);
		} else {
			bot.stopLiftMotor();
		}


		// Extend manipulator arm up and down with gamepad 2 right and left trigger
		if (gamepad2.right_trigger > 0) {
			bot.extendArm(gamepad2.right_trigger * manipulatorScaleFactor);
		} else if (gamepad2.left_trigger > 0) {
			bot.retractArm(gamepad2.left_trigger * manipulatorScaleFactor);
		} else {
			bot.stopExtendMotor();
		}


		// Control grab wheels with gamepad 2 dpad--up and down, with L/R stopping the moter
		if (gamepad2.dpad_up) {
			bot.grabBlock(manipulatorScaleFactor);
		} else if (gamepad2.dpad_down) {
			bot.releaseBlock(manipulatorScaleFactor);
		} else if (gamepad2.dpad_left || gamepad2.dpad_right) {
			bot.stopGrabMotor();
		}


		telemetry.addData("Status", "Run Time: " + runtime.toString());
		telemetry.addData("Drive Scale", "%.2f", driveScaleFactor);
		telemetry.addData("Manipulator Scale", "%.2f", manipulatorScaleFactor);
/*
		telemetry.addData("Lift Position", "%d", bot.liftMotor.getCurrentPosition());
		telemetry.addData("Tilt Position", "%d", bot.tiltMotor.getCurrentPosition());
		telemetry.addData("Extend Position", "%d", bot.extendMotor.getCurrentPosition());
*/

	}

	/*
      * Code to run ONCE after the driver hits STOP
      */
	@Override
	public void stop() {
	}

}
