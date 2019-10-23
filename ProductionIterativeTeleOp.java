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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Test: Control Tester", group="Iterative Opmode")

public class ProductionIterativeTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DefenderBot productionBot = new DefenderBot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
	@Override
	public void init() {
		productionBot.init(hardwareMap, new ProductionTestConfiguration());

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = gamepad1.left_stick_x;

        double rightStickY = -gamepad1.right_stick_y;
        double rightStickX = gamepad1.right_stick_x;

	   if (gamepad1.start) {
	   		productionBot.stopAllMotors();
	   }

	   if (gamepad1.y) {
	        telemetry.addData("lift", "going up");
		   productionBot.moveLiftUp(0.5);
	   } else if (gamepad1.a) {
	        telemetry.addData("lift", "going down");
		   productionBot.moveLiftDown(0.5);
	   }
	   if (gamepad1.x) {
	        telemetry.addData("arm", "extending");
		   productionBot.extendArm(0.5);
	   } else if (gamepad1.b) {
	        telemetry.addData("arm", "retracting");
		   productionBot.retractArm(0.5);
	   }

	   if (rightStickY > 0) {
		   productionBot.grabBlock(1);
	   } else if (rightStickY < 0) {
	   productionBot.releaseBlock(1);

		   } else {
		   productionBot.grabBlock(0);

		   }

		if (gamepad1.dpad_up && gamepad1.dpad_left) {
			productionBot.driveDiagonalForwardLeft(1,1);
		} else if (gamepad1.dpad_up && gamepad1.dpad_right) {
		   	productionBot.driveDiagonalForwardRight(1,1);
		   } else if (gamepad1.dpad_down && gamepad1.dpad_left) {
			   productionBot.driveDiagonalBackwardLeft(1,1);
		   } else if (gamepad1.dpad_down && gamepad1.dpad_right) {
			   productionBot.driveDiagonalBackwardRight(1,1);
		   } else if (rightStickX > 0) {
			   productionBot.driveRight(rightStickX, rightStickX);
		   } else if (rightStickX < 0) {
			   productionBot.driveLeft(-1 * rightStickX, -1 * rightStickX);
		   } else if (leftStickX == 0) {
				if (leftStickY > 0) {
					productionBot.driveForward(leftStickY, leftStickY);
				} else if (leftStickY < 0) {
					productionBot.driveBackward(-1 * leftStickY, -1 * leftStickY);
				} else {
					productionBot.stopDriving();
				}
			} else if (leftStickX > 0) {
				productionBot.turnClockwise(leftStickX, leftStickX);
			} else if (leftStickX < 0) {
				productionBot.turnCounterClockwise(-1 * leftStickX, -1 * leftStickX);
			} else {
				productionBot.stopDriving();
			}

/*
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
*/

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Controller", "left stick y (%.2f), left stick x (%.2f)", leftStickY, leftStickX);
        telemetry.addData("FL Encoder", "(%03d)", productionBot.frontLeftDrive.getCurrentPosition());

        telemetry.addData("Left Front", "%.2f", productionBot.frontLeftDrive.getPower());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
