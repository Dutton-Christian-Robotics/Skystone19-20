package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class InternalGyroscopeService implements Runnable {


	private boolean isRunning = true;
	private int sleepTime;
	private BNO055IMU imu;
	private Orientation orientation;
	private Axis headingAxis;

	public InternalGyroscopeService(BNO055IMU i, int st, Axis h) {
		imu = i;
		sleepTime = st;
		headingAxis = h;
	}

	@Override
	public void run() {
		while (isRunning) {
			orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public double xAngle() {
		return orientation.firstAngle;
	}

	public double yAngle() {
		return orientation.secondAngle;
	}

	public double zAngle() {
		return orientation.thirdAngle;
	}

	public double heading() {
		double angle;
		switch (headingAxis) {
			case X:
				angle = xAngle();
				break;
			case Y:
				angle = yAngle();
				break;
			case Z:
				angle = zAngle();
				break;
			default:
				angle = xAngle();
		}
		return angle;
	}

	public void stop() {
		isRunning = false;
	}

 	public enum Axis {
	 	X,
	 	Y,
	 	Z
 	}


}