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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class SkystoneVisionService implements Runnable {

	private boolean isRunning = true;
	private int sleepTime;

	private VuforiaLocalizer vuforia = null;
	private VuforiaTrackables targets;
	private VuforiaTrackable skystoneTarget;

	private WebcamName webcamName;

	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
	private static final boolean PHONE_IS_PORTRAIT = false  ;

	// Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
	// We will define some constants and conversions here
	private static final float mmPerInch        = 25.4f;
	private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

	// Constant for Stone Target
	private static final float stoneZ = 2.00f * mmPerInch;

	// Constants for the center support targets
	private static final float bridgeZ = 6.42f * mmPerInch;
	private static final float bridgeY = 23 * mmPerInch;
	private static final float bridgeX = 5.18f * mmPerInch;
	private static final float bridgeRotY = 59;                                 // Units are degrees
	private static final float bridgeRotZ = 180;

	// Constants for perimeter targets
	private static final float halfField = 72 * mmPerInch;
	private static final float quadField  = 36 * mmPerInch;

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

		webcamName = hwMap.get(WebcamName.class, "WEBCAM"); //add name to config file
		VuforiaLocalizer.Parameters parameters;

// 		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		if (cmvid == -1) {
			parameters = new VuforiaLocalizer.Parameters();
		} else {
			parameters = new VuforiaLocalizer.Parameters(cmvid);
		}
		parameters.vuforiaLicenseKey = botConfiguration.vuforiaKey;
// 		parameters.cameraDirection   = CAMERA_CHOICE;
		parameters.cameraName = webcamName;

		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("Skystone");

		skystoneTarget = targets.get(0);
		skystoneTarget.setName("Skystone Target");

		skystoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		// For convenience, gather together all the trackable objects in one easily-iterable collection */
		List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
		allTrackables.addAll(targets);

		// We need to rotate the camera around it's long axis to bring the correct camera forward.
// 		if (CAMERA_CHOICE == BACK) {
// 			phoneYRotate = -90;
/*
		} else {
			phoneYRotate = 90;
		}
*/

		// Rotate the phone vertical about the X axis if it's in portrait mode
// 		if (PHONE_IS_PORTRAIT) {
// 			phoneXRotate = 9-0 ;	//don't need this for webcam?
// 		}

		// Next, translate the camera lens to where it is on the robot.
		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
		final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
		final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

		OpenGLMatrix robotFromCamera = OpenGLMatrix
			.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
			.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

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
				movingAverageItems.clear();
			}
			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

    //--------------------------------------------------------------------------------------------

    public int foo() {
	    return movingAverageItems.size();
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

				dXsum += translation.get(0) / mmPerInch;
				dYsum += translation.get(1) / mmPerInch;
				dZsum += translation.get(2) / mmPerInch;
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
