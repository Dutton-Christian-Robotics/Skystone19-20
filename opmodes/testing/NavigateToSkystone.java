package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Testing: Nav to Skystone", group ="Testing")

public class NavigateToSkystone extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();
	private SkystoneBot bot = new SkystoneBot();
	private boolean foundSkystone = false;
	private boolean atSkystone = false;
	private boolean gotSkystone = false;
	private double dX;
	private double dY;
	private double rZ;


    @Override
    public void runOpMode() {

		telemetry.addData("Status", "Initializing");
		telemetry.update();
		bot.init(hardwareMap, new ProductionTestConfiguration());
		telemetry.addData("Status", "Initialized, waiting for IMU");
		telemetry.update();
		try {
			bot.activateInternalGyroscopeService();
		} catch (Exception e) {
			telemetry.addData("Status", "IMU failure");
			telemetry.update();
		}
		telemetry.addData("Status", "Initialized, waiting for Vision");
		telemetry.update();
		bot.activateSkystoneVisionService();
		telemetry.addData("Status", "Visition Activated, ready for Start");
		telemetry.update();

		waitForStart();
		telemetry.addData("Status", "Moving Forward");
		telemetry.update();
// 		bot.manipulatorsToCapturePosition();
		bot.driveForward(1,1);
		sleep(700);
		bot.stopDriving();

		while (opModeIsActive()) {

			telemetry.addData("Run Time", runtime.toString());
// 			telemetry.update();

			if (!atSkystone && bot.skystoneVisionService.isSkystoneVisible()) {
// 			if (!foundSkystone && bot.skystoneVisionService.isSkystoneVisible()) {
				telemetry.addData("Skystone", "Yes");
				telemetry.update();

				foundSkystone = true;
				SimpleMovementSettings slide;

	/*
				dX = -10;
				dY = 15;
				rZ = -15;
	*/


				dX = bot.skystoneVisionService.dX;
				dY = bot.skystoneVisionService.raw_dY();
				rZ = bot.skystoneVisionService.rZ + 90;

				while (Math.abs(dY) > 4) {
					telemetry.addData("Status", "Adjusting position to match stone");

					if (dY > 0) {
						telemetry.addData("Strafe", "right");
						telemetry.addData("Nav", "%.2f %.2f %.2f", dX, dY, rZ);
						telemetry.update();
						bot.driveRight(0.25, 0.25);
						sleep(300);
						bot.stopDriving();

					} else if (dY < 0) {
						telemetry.addData("Strafe", "left");
						telemetry.addData("Nav", "%.2f %.2f %.2f", dX, dY, rZ);
						telemetry.update();
						bot.driveLeft(0.25, 0.25);
						sleep(300);
						bot.stopDriving();

					} else {
						telemetry.addData("Strafe", "none: %.2f", dY);

					}
// 				while ((Math.abs(dX) > 15) || (Math.abs(dY) > 5) || (Math.abs(rZ) > 5)) {
/*

					double percentSideways = (90 - Math.abs(rZ) - Math.toDegrees(Math.atan(Math.abs(dX) / Math.abs(dY)))) / 90;

					if (dY < 0) {
						slide = SimpleMovementSettings.combine(bot.forwardMotion, bot.slideLeftMotion, percentSideways);
					} else {
						slide = SimpleMovementSettings.combine(bot.forwardMotion, bot.slideRightMotion, percentSideways);
					}


					if (rZ < 0) {
						bot.driveBasedOnMovementSettings(slide.scale(0.15));
// 						bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(slide, bot.turnLeftMotion, Math.abs(rZ) / 90));

					} else {
						bot.driveBasedOnMovementSettings(slide.scale(0.15));
// 						bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(slide, bot.turnRightMotion, Math.abs(rZ) / 90));

					}
					sleep(400);
					bot.stopDriving();
					sleep(500);
*/
					try {
						dX = bot.skystoneVisionService.dX;
						dY = bot.skystoneVisionService.raw_dY();
						rZ = bot.skystoneVisionService.rZ+90;
					} catch (Exception e) {
						telemetry.addData("Status", "coordinates failure");
						telemetry.update();
					}
					telemetry.addData("Nav", "%.2f %.2f %.2f", dX, dY, rZ);
					telemetry.update();

				}
				atSkystone = true;
				telemetry.addData("Nav", "%.2f %.2f %.2f", dX, dY, rZ);
				telemetry.update();


				// NEXT STEPS
				// * create loop, check new values, stop when at block
				// * incorporate rotation
			} else if (foundSkystone && atSkystone && !gotSkystone) {
				telemetry.addData("Status", "At Skystone, ready for capture");
				telemetry.update();
				bot.manipulatorsToPreCapturePosition();
				bot.driveForward(0.3, 0.3);
				sleep(375);
				bot.stopDriving();
				bot.manipulatorsToCapturePosition();
				bot.captureBlock();
				gotSkystone = true;
				bot.manipulatorsToTravelPosition();
				try {
					bot.comeToHeading(InternalGyroscopeService.Direction.W);
				} catch (Exception e) {
					telemetry.addData("Status", "cannot rotate due to IMU failure");
					telemetry.update();

				}
				bot.driveForward(1,1);
				sleep(700);
/*
				bot.turnCounterClockwise(1,1);
				sleep(400);
*/
				bot.stopDriving();
			} else if (!foundSkystone && !atSkystone && !gotSkystone) {
				telemetry.addData("Status", "Search strafe");
				telemetry.update();
				bot.driveRight(0.25, 0.25);
				sleep(500);
				bot.stopDriving();
				sleep(750);
			}
			bot.stopDriving();
		}

	}
}