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

public class SimpleImuTest extends LinearOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	private SkystoneBot bot = new SkystoneBot();


    @Override
    public void runOpMode() {
		bot.init(hardwareMap, new ProductionTestConfiguration());

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		while (!isStopRequested() && !bot.imu.isGyroCalibrated()) {
			sleep(50);
			idle();
		}
		telemetry.addData("Status", "IMU calibrated");
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();

		while (opModeIsActive()) {
			telemetry.addData("X Axis", bot.internalGyroscopeService.xAngle());
			telemetry.addData("Y Axis", bot.internalGyroscopeService.yAngle());
			telemetry.addData("Z Axis", bot.internalGyroscopeService.zAngle());

			telemetry.addData("Heading", bot.internalGyroscopeService.heading());

			telemetry.addData("Java", Runtime.class.getPackage().getImplementationVersion());
// 			telemetry.addData("Java", System.getProperty("java.version"));


/*
			AngularVelocity omegas = bot.imu.getAngularVelocity();
			omegas.toAngleUnit(AngleUnit.DEGREES);
			telemetry.addData("angular v x:", String.format("%f", omegas.xRotationRate));
			telemetry.addData("angular v y:", String.format("%f", omegas.yRotationRate));
			telemetry.addData("angular v z:", String.format("%f", omegas.zRotationRate));
			telemetry.addData("in motion", bot.isInMotionAccordingToImu() ? "yes" : "no");
*/
			telemetry.update();

			if (bot.internalGyroscopeService.heading() < -10 && bot.internalGyroscopeService.heading() > -180) {
				bot.turnCounterClockwise(0.2, 0.2);
				sleep(25);
			} else if (bot.internalGyroscopeService.heading() > 10 && bot.internalGyroscopeService.heading() < 180) {
				bot.turnClockwise(0.2, 0.2);
				sleep(25);
			} else {
				bot.stopDriving();
			}


		}
		bot.shutdown();
	}
}
