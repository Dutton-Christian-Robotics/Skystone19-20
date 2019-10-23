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
@Autonomous(name="Basic: Production Drive Test Linear OpMode", group="Linear Opmode")

public class ProductionDriveTesterLinear extends LinearOpMode {

    // Declare OpMode members.
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

		// Wait for the game to start (driver presses PLAY)
		waitForStart();
		runtime.reset();


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
