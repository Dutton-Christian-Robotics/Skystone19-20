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
@Autonomous(name="Beta: Autonomous Blue 4", group="Linear Opmode")

public class AutonomousBlueFourOpMode extends LinearOpMode {

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

		// raise the arm to allow trapping
		bot.moveLiftUp(1);
		bot.extendArm(0.75);

		// begin driving forward while arm is going up
		bot.driveForward(0.5, 0.5);

		sleep(600);
		bot.stopLiftMotor();
		sleep(200);
		bot.stopExtendMotor();
		sleep(1200);
		bot.stopDriveMotors();

		bot.moveLiftDown(0.75);
		bot.grabBlock(1);
		sleep(600);
		bot.retractArm(0.75);
		sleep(400);
		bot.stopGrabMotor();
		bot.stopExtendMotor();
		bot.stopLiftMotor();

		bot.driveDiagonalBackwardLeft(1,1);
		sleep(1600);

		bot.turnCounterClockwise(0.5, 0.5);
		sleep(100);
		bot.moveLiftUp(1);
		sleep(700);
		bot.stopLiftMotor();
		bot.driveForward(0.5, 0.5);
		sleep(2750);

		bot.stopDriveMotors();
		bot.releaseBlock(0.5);
		sleep(400);
		bot.stopGrabMotor();

		bot.moveLiftUp(1);
		sleep(600);
		bot.stopLiftMotor();

		bot.driveBackward(0.5, 0.5);
		sleep(2400);
		bot.stopDriveMotors();
		bot.moveLiftDown(0.5);
		sleep(400);
		bot.stopLiftMotor();

    }
}
