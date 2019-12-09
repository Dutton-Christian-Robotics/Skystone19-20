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

@TeleOp(name="Testing: Nav to Skystone", group ="Linear Opmode")

public class NavigateToSkystone extends LinearOpMode {
	private ElapsedTime runtime = new ElapsedTime();
	private SkystoneBot bot = new SkystoneBot();
	private boolean foundSkystone = false;
	private double dX;
	private double dY;
	private double rZ;


    @Override
    public void runOpMode() {

		telemetry.addData("Status", "Initializing");
		telemetry.update();
		bot.init(hardwareMap, new ProductionTestConfiguration());
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		bot.activateSkystoneVisionService();

		waitForStart();


		while (opModeIsActive()) {

			telemetry.addData("Status", "Run Time: " + runtime.toString());
			telemetry.update();

			if (bot.skystoneVisionService.isSkystoneVisible()) {
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
				dY = bot.skystoneVisionService.dY;
				rZ = bot.skystoneVisionService.rZ + 90;

				while ((Math.abs(dX) > 3) || (Math.abs(dY) > 3) || (Math.abs(rZ) > 5)) {
					telemetry.addData("Nav", "%.2f %.2f %.2f", dX, dY, rZ);
					telemetry.update();

					double percentSideways = (90 - Math.abs(rZ) - Math.toDegrees(Math.atan(Math.abs(dX) / Math.abs(dY)))) / 90;

					if (dY < 0) {
						slide = SimpleMovementSettings.combine(bot.forwardMotion, bot.slideLeftMotion, percentSideways);
					} else {
						slide = SimpleMovementSettings.combine(bot.forwardMotion, bot.slideRightMotion, percentSideways);
					}


					if (rZ < 0) {
						bot.driveBasedOnMovementSettings(slide);
// 						bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(slide, bot.turnLeftMotion, Math.abs(rZ) / 90));

					} else {
						bot.driveBasedOnMovementSettings(slide);
// 						bot.driveBasedOnMovementSettings(SimpleMovementSettings.combine(slide, bot.turnRightMotion, Math.abs(rZ) / 90));

					}
					sleep(500);
					dX = bot.skystoneVisionService.dX;
					dY = bot.skystoneVisionService.dY;
					rZ = bot.skystoneVisionService.rZ+90;

				}


				// NEXT STEPS
				// * create loop, check new values, stop when at block
				// * incorporate rotation
			}
			bot.stopDriving();
		}

	}
}