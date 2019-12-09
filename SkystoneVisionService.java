package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SkystoneVisionService implements Runnable {

	private boolean isRunning = true;
	private int sleepTime;

	private VuforiaLocalizer vuforia = null;
	private VuforiaTrackables targets;
	private VuforiaTrackable skystoneTarget;

	private WebcamName webcamName;


	private float phoneXRotate    = 0;
	private float phoneYRotate    = 0;
	private float phoneZRotate    = 0;


	private static final int movingAverageCount = 10;
	private List<OpenGLMatrix> movingAverageItems;
	public float dX;
	public float dY;
	public float dZ;
	public float rX;
	public float rY;
	public float rZ;


	public SkystoneVisionService(int st, DefenderBotConfiguration botConfiguration, HardwareMap hwMap, int cmvid) {
		sleepTime = st;

		VuforiaLocalizer.Parameters parameters;

		if (cmvid == -1) {
			parameters = new VuforiaLocalizer.Parameters();
		} else {
			parameters = new VuforiaLocalizer.Parameters(cmvid);
		}
		parameters.vuforiaLicenseKey = botConfiguration.vuforiaKey;

		if (botConfiguration.useExternalWebcam) {
			webcamName = hwMap.get(WebcamName.class, botConfiguration.externalWebcamName);
			parameters.cameraName = webcamName;
		} else {
			parameters.cameraDirection   = botConfiguration.cameraChoice;

		}


		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("Skystone");

		skystoneTarget = targets.get(0);
		skystoneTarget.setName("Skystone Target");

		skystoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, Measurements.stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		// For convenience, gather together all the trackable objects in one easily-iterable collection */
		List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
		allTrackables.addAll(targets);

		OpenGLMatrix robotFromCamera = OpenGLMatrix
			.translation(botConfiguration.webcam_dX, botConfiguration.webcam_dY, botConfiguration.webcam_dZ)
			.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, botConfiguration.webcam_rX, botConfiguration.webcam_rZ, botConfiguration.webcam_rY));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, robotFromCamera);
// 			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}

		movingAverageItems = new ArrayList<OpenGLMatrix>();

		targets.activate();


	}

    //--------------------------------------------------------------------------------------------

	@Override
	public void run() {
		while (isRunning) {
			//do stuff here
			if (isSkystoneVisible()) {
				if (movingAverageItems.size() >= movingAverageCount) {
					movingAverageItems.remove(0);
				}
				movingAverageItems.add(rawDirectionToSkystone());
				calculateMovingAverage();
			} else {
				clearMovingAverage();
			}
			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

    //--------------------------------------------------------------------------------------------

	public void clearMovingAverage() {
		movingAverageItems.clear();
	}


	public void calculateMovingAverage() {
		if (movingAverageItems.size() > 0) {
			float dXsum = 0;
			float dYsum = 0;
			float dZsum = 0;
			float rXsum = 0;
			float rYsum = 0;
			float rZsum = 0;

			Iterator itr = movingAverageItems.iterator();
			while(itr.hasNext()) {
				OpenGLMatrix location = (OpenGLMatrix)itr.next();
				VectorF translation = location.getTranslation();
				Orientation rotation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);

				dXsum += translation.get(0) / Measurements.mmPerInch;
				dYsum += translation.get(1) / Measurements.mmPerInch;
				dZsum += translation.get(2) / Measurements.mmPerInch;
				rXsum += rotation.firstAngle;
				rYsum += rotation.secondAngle;
				rZsum += rotation.thirdAngle;
			}

			dX = dXsum / movingAverageItems.size();
			dY = dYsum / movingAverageItems.size();
			dZ = dZsum / movingAverageItems.size();
			rX = rXsum / movingAverageItems.size();
			rY = rYsum / movingAverageItems.size();
			rZ = rZsum / movingAverageItems.size();


		} else {
			dX = 0;
			dY = 0;
			dZ = 0;
			rX = 0;
			rY = 0;
			rZ = 0;
	    }
    }

    //--------------------------------------------------------------------------------------------

	public boolean isSkystoneVisible() {
		return ((VuforiaTrackableDefaultListener)skystoneTarget.getListener()).isVisible();
	}

	//--------------------------------------------------------------------------------------------

	public OpenGLMatrix rawDirectionToSkystone() {
		return ((VuforiaTrackableDefaultListener)skystoneTarget.getListener()).getFtcFieldFromRobot();
	}

    //--------------------------------------------------------------------------------------------

	public void stop() {
		isRunning = false;
	}

}
