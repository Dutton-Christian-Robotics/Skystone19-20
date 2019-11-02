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

import java.util.Random;

/**
 * This op mode is currently used for testing functionality *
 */
@Autonomous(name="Game: Autonomous Red 1", group="Linear Opmode")

public class AutonomousRedOneOpMode extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private DefenderBot bot = new DefenderBot();

    @Override
    public void runOpMode() {
		bot.init(hardwareMap, new ProductionTestConfiguration());

		telemetry.addData("Status", "Initialized");
		telemetry.update();


		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		// drive diagonally forward left
		bot.driveBasedOnMovementSettings(bot.diagonalForwardRightMotion);
		sleep(1500);

		// drive a little bit more left
		bot.driveRight(0.5, 0.5);
		sleep(1200);

		// drive forward to the edge of the building platform
		bot.driveForward(0.25, 0.25);
		sleep(2000);
		bot.stopDriveMotors();

		// extend the grabber arm
		bot.extendArm(1);
		sleep(2000);
		bot.stopExtendMotor();

		// lower the arm to grab the platform
		bot.tiltArmDown(0.25);
		sleep(800);
		bot.stopTiltMotor();

		// pull the platform backwards
		bot.driveBackward(0.5, 0.5);
		sleep(3100);
		bot.stopDriveMotors();

		// release the platform
		bot.tiltArmUp(0.75);
		sleep(750);
		bot.stopTiltMotor();

		// retract the arm--I think it helps us maneuver better
		bot.retractArm(0.5);
		sleep(1500);
		bot.stopExtendMotor();


		// slide out from behind the platform
		bot.driveLeft(0.5, 0.5);
		sleep(2500);

		// get away from the wall a bit
		bot.driveForward(0.5, 0.5);
		sleep(800);

		// rive to the mid-field line
		bot.driveLeft(0.5, 0.5);
		sleep(1100);


    }
}
