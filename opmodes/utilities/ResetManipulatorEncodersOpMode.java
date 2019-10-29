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


@TeleOp(name="Utility: Reset Encoders", group="Iterative Opmode")

public class ResetManipulatorEncodersOpMode extends OpMode {
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
		double leftStickY = -gamepad1.left_stick_y;
		double leftStickX = gamepad1.left_stick_x;

		double rightStickY = -gamepad1.right_stick_y;
		double rightStickX = gamepad1.right_stick_x;



	   if (rightStickY > 0) {
		   productionBot.tiltArmUp(rightStickY);
	   } else if (rightStickY < 0) {
		   productionBot.tiltArmDown(rightStickY);
	   } else {
		   productionBot.tiltArmDown(0);
	   }

	   if (gamepad1.y) {
	        telemetry.addData("lift", "going up");
		   productionBot.moveLiftUp(0.5);
	   } else if (gamepad1.a) {
	        telemetry.addData("lift", "going down");
		   productionBot.moveLiftDown(0.5);
	   } else {
		   productionBot.stopLiftMotor();
	   }

	   if (gamepad1.x) {
	        telemetry.addData("arm", "extending");
		   productionBot.extendArm(0.5);
	   } else if (gamepad1.b) {
	        telemetry.addData("arm", "retracting");
		   productionBot.retractArm(0.5);
	   } else {
		   productionBot.stopExtendMotor();
	   }
	   if (gamepad1.dpad_up) {
		   productionBot.grabBlock(1);
	   } else if (gamepad1.dpad_down) {
		   productionBot.releaseBlock(1);
	   } else {
		   productionBot.stopGrabMotor();
	   }

	   if (gamepad1.start) {
		   productionBot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		   productionBot.tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		   productionBot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	        telemetry.addData("Encoders", "resetting");
	        try {
		        Thread.sleep(3000);
	        } catch (Exception e) {

	        }
		   productionBot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		   productionBot.tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		   productionBot.extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	   }


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Lift Position", "%d", productionBot.liftMotor.getCurrentPosition());
        telemetry.addData("Tilt Position", "%d", productionBot.tiltMotor.getCurrentPosition());
        telemetry.addData("Extend Position", "%d", productionBot.extendMotor.getCurrentPosition());
/*
        telemetry.addData("Controller", "left stick y (%.2f), left stick x (%.2f)", leftStickY, leftStickX);
        telemetry.addData("FL Encoder", "(%03d)", productionBot.frontLeftDrive.getCurrentPosition());

        telemetry.addData("Left Stick Y", "%.2f", leftStickY);
*/


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
