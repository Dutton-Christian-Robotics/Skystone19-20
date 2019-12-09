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

@TeleOp(name="Testing: 1 Gamepad Control Tester", group="Iterative Opmode")

public class OneGamepadControlTesterOpMode extends OpMode {
	private ElapsedTime runtime = new ElapsedTime();
	private SkystoneBot bot = new SkystoneBot();
	private double driveScaleFactor = 1;
	private Debouncer decreaseDriveScaleFactorDebouncer;

    /*
     * Code to run ONCE when the driver hits INIT
     */
	@Override
	public void init() {
		telemetry.addData("Status", "Initializing");
		telemetry.update();
		bot.init(hardwareMap, new ProductionTestConfiguration());
/*
		while (!bot.imu.isGyroCalibrated()) {
			try {
				Thread.sleep(50);
			} catch (Exception e) {

			}
		}
		telemetry.addData("Status", "IMU calibrated");
		telemetry.update();
*/

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
		decreaseDriveScaleFactorDebouncer = new Debouncer(200, new Callable<Void>() {
			@Override
			public Void call() {
				driveScaleFactor -= 0.25;
				return null;
			}
		});
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
		double leftStickY = -gamepad1.left_stick_y;
		double leftStickX = gamepad1.left_stick_x;

		double rightStickY = -gamepad1.right_stick_y;
		double rightStickX = gamepad1.right_stick_x;



	    if (gamepad1.y) {
	    		bot.openClaw();
	    		telemetry.addData("claw", "opening");

	    } else if (gamepad1.x) {
	    		bot.neutralClaw();
	    		telemetry.addData("claw", "neutral");

	    } else if (gamepad1.b) {
		    bot.releaseBlock();
	    		telemetry.addData("claw", "release");

	    } else if (gamepad1.a) {
		    bot.captureBlock();
	    		telemetry.addData("claw", "capture");
	    }

		if (gamepad1.left_bumper) {
// 			bot.setExtendPosition(300);
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

		if (gamepad1.start) {
			bot.stopAllMotors();

		} else if (gamepad1.left_trigger > 0) {
			bot.driveLeft(gamepad1.left_trigger * driveScaleFactor, gamepad1.left_trigger * driveScaleFactor);

		} else if (gamepad1.right_trigger > 0) {
			bot.driveRight(gamepad1.right_trigger * driveScaleFactor, gamepad1.right_trigger * driveScaleFactor);

		} else if (gamepad1.dpad_left) {
		    bot.grabFoundation();

		} else if (gamepad1.dpad_right) {
		    bot.releaseFoundation();



/*
		} else if (gamepad1.dpad_up && gamepad1.dpad_left) {
			bot.driveBasedOnMovementSettings(bot.diagonalForwardLeftMotion.scale(driveScaleFactor));
*/

/*
		} else if (gamepad1.dpad_up && gamepad1.dpad_right) {
			bot.driveBasedOnMovementSettings(bot.diagonalForwardRightMotion.scale(driveScaleFactor));
*/

/*
		} else if (gamepad1.dpad_down && gamepad1.dpad_left) {
			bot.driveBasedOnMovementSettings(bot.diagonalReverseLeftMotion.scale(driveScaleFactor));
*/

/*
		} else if (gamepad1.dpad_down && gamepad1.dpad_right) {
			bot.driveBasedOnMovementSettings(bot.diagonalReverseRightMotion.scale(driveScaleFactor));
*/

		} else if (leftStickY > 0) {

	   		// Forward and Right
			if (leftStickX > 0) {
				SimpleMovementSettings foo = SimpleMovementSettings.combine(bot.forwardMotion, bot.turnRightMotion, leftStickX, leftStickY).scale(driveScaleFactor);
// 				telemetry.addData("movement", foo.toString());
				bot.driveBasedOnMovementSettings(foo);

			// Forward and Left
			} else if (leftStickX < 0) {
				SimpleMovementSettings foo = SimpleMovementSettings.combine(bot.forwardMotion, bot.turnLeftMotion, leftStickX, leftStickY).scale(driveScaleFactor);
// 				telemetry.addData("movement", foo.toString());
				bot.driveBasedOnMovementSettings(foo);

			// Just Forward
			} else {
// 				telemetry.addData("movement", bot.forwardMotion.scale(leftStickY).toString());
				bot.driveBasedOnMovementSettings(bot.forwardMotion.scale(leftStickY * driveScaleFactor));
			}


		} else if (leftStickY < 0) {

			// Reverse and Right
			if (leftStickX > 0) {
				SimpleMovementSettings foo = SimpleMovementSettings.combine(bot.reverseMotion, bot.turnRightMotion, leftStickX, leftStickY);
// 				telemetry.addData("movement", foo.toString());
				bot.driveBasedOnMovementSettings(foo.scale(driveScaleFactor));

			// Reverse and Left
			} else if (leftStickX < 0) {
				SimpleMovementSettings foo = SimpleMovementSettings.combine(bot.reverseMotion, bot.turnLeftMotion, leftStickX, leftStickY);
// 				telemetry.addData("movement", foo.toString());
				bot.driveBasedOnMovementSettings(foo.scale(driveScaleFactor));

			// Just Reverse
			} else {
// 				telemetry.addData("movement", bot.reverseMotion.scale(leftStickY).toString());
				bot.driveBasedOnMovementSettings(bot.reverseMotion.scale(leftStickY * driveScaleFactor));
			}




		} else if (leftStickX > 0) {
			SimpleMovementSettings foo = bot.turnRightMotion.scale(leftStickX * driveScaleFactor);
			telemetry.addData("movement", foo.toString());
			bot.driveBasedOnMovementSettings(foo);


		} else if (leftStickX < 0) {
			SimpleMovementSettings foo = bot.turnLeftMotion.scale(leftStickX * driveScaleFactor);
			telemetry.addData("movement", foo.toString());
			bot.driveBasedOnMovementSettings(foo);
		} else {
			bot.driveBasedOnMovementSettings(bot.stopMotion);
		}

		if (gamepad1.dpad_up) {
			bot.moveLiftUp(1);
		} else if (gamepad1.dpad_down) {
			bot.moveLiftDown(1);
		} else {
			bot.stopLiftMotor();
		}

	   if (rightStickY > 0) {
		   bot.tiltArmUp(rightStickY);
	   } else if (rightStickY < 0) {
		   bot.tiltArmDown(rightStickY);
	   } else {
		   bot.tiltArmDown(0);
	   }

	   if (rightStickX > 0) {
		   bot.extendArm(rightStickX);
	   } else if (rightStickX < 0) {
		   bot.retractArm(rightStickX);
	   } else {
		   bot.stopExtendMotor();
	   }


/*
	   if (gamepad1.y) {
	        telemetry.addData("lift", "going up");
		   bot.moveLiftUp(0.5);
	   } else if (gamepad1.a) {
	        telemetry.addData("lift", "going down");
		   bot.moveLiftDown(0.5);
	   } else {
		   bot.stopLiftMotor();
	   }
*/

/*
	   if (gamepad1.x) {
	        telemetry.addData("arm", "extending");
		   bot.extendArm(0.5);
	   } else if (gamepad1.b) {
	        telemetry.addData("arm", "retracting");
		   bot.retractArm(0.75);
	   } else {
		   bot.stopExtendMotor();
	   }
	   if (gamepad1.dpad_up) {
		   bot.grabBlock(1);
	   } else if (gamepad1.dpad_down) {
		   bot.releaseBlock(1);
	   } else {
		   bot.stopGrabMotor();
	   }
*/


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Scale", "%.2f", driveScaleFactor);
        telemetry.addData("Lift Position", "%d", bot.liftMotor.getCurrentPosition());
        telemetry.addData("Tilt Position", "%d", bot.tiltMotor.getCurrentPosition());
        telemetry.addData("Extend Position", "%d", bot.extendMotor.getCurrentPosition());
        telemetry.update();
/*
        telemetry.addData("FL Encoder", "(%03d)", bot.frontLeftDrive.getCurrentPosition());

*/


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
