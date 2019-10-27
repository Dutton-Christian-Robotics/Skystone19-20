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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;

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
@Autonomous(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class DriveForwardIterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private long start = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "Front Left Orange");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "Front Right Red");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "Rear Left Camo");
        rearRightDrive = hardwareMap.get(DcMotor.class, "Rear Right Green");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);

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
        start = System.currentTimeMillis();
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

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        leftPower    = 1.0 ;
        rightPower   = 1.0 ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


        long elapsedTimeLong = System.currentTimeMillis() - start;
        String elapsedTimeString = Long.toString(elapsedTimeLong /1000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + elapsedTimeString);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);



        // Drive until couch, then stop
        if (elapsedTimeLong > 3350)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;

        }

        // Turn Right
        if (elapsedTimeLong > 5000)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
            leftPower    = 1.0 ;
            rightPower   = 1.0 ;
        }

        // Stop Turning
        if (elapsedTimeLong > 5575)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }

        // Drive by front of couch
        if (elapsedTimeLong > 7000)
        {
            leftPower    = 1.0 ;
            rightPower   = 1.0 ;
        }

        // Stop in front of couch
        if (elapsedTimeLong > 9200)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }


        // Start Left turn at front of couch
        if (elapsedTimeLong > 11000)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftPower    = 1.0 ;
            rightPower   = 1.0 ;
        }

        // Stop turn at front of couch
        if (elapsedTimeLong > 11475)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }

        // Start driving at side of couch
        if (elapsedTimeLong > 13000)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftPower    = 1.0;
            rightPower   = 1.0 ;
        }

        // Stop driving at side of couch
        if (elapsedTimeLong > 14800)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }

        // Start turning at side of couch
        if (elapsedTimeLong > 16000)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftPower    = 1.0 ;
            rightPower   = 1.0 ;
        }


        // Stop turning at side of couch
        if (elapsedTimeLong > 16550)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }

        // Start driving in front of TV
        if (elapsedTimeLong > 18000)
        {
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftPower    = 1.0;
            rightPower   = 1.0 ;
        }

        // Stop driving at front of TV
        if (elapsedTimeLong > 19800)
        {
            leftPower    = 0.0 ;
            rightPower   = 0.0 ;
        }

        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        rearLeftDrive.setPower(leftPower);
        rearRightDrive.setPower(rightPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
