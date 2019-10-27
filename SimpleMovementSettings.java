package org.firstinspires.ftc.teamcode;

import java.lang.Math;


/**
 * This class id designed to help facilitate complex motions, such as moving forward and turning at the same time.
 * Essentially the class consists of four properties, a power setting for each of the four drive wheels. Unlike
 * other places in the code, here power is treated as -1 to 1. This allows us to capure capture the required wheel
 * movements with one value--power--instead of a 0-1 power and a direction name (forward or reverse).
 *
 * The expectaion is that most times this class would have values of -1, 0, or 1 for the values. A "scale" method
 * can reduce power by a percentage and then returns a new instance.
 *
 * The combine functions attempt to blend the drive power requirements for two different movement types, resulting in
 * a new hybrid movement--such as turning while moving forward.
 *
 */

public class SimpleMovementSettings {

	public double frontLeftDrive = 0;
	public double rearLeftDrive = 0;
	public double frontRightDrive = 0;
	public double rearRightDrive = 0;

	public SimpleMovementSettings(double fl, double rl, double fr, double rr) {
		frontLeftDrive = fl;
		rearLeftDrive = rl;
		frontRightDrive = fr;
		rearRightDrive = rr;
	}

	public static SimpleMovementSettings combine(SimpleMovementSettings movementA, SimpleMovementSettings movementB, double combinationPercentage) {
		if (combinationPercentage > 1) {
			combinationPercentage /= 100;
		}
		SimpleMovementSettings resultingMovementSettings = new SimpleMovementSettings(0,0,0,0);

		resultingMovementSettings.frontLeftDrive	= Range.clip(movementA.frontLeftDrive + combinationPercentage * (movementB.frontLeftDrive - movementA.frontLeftDrive), -1.0, 1.0);
		resultingMovementSettings.rearLeftDrive		= Range.clip(movementA.rearLeftDrive + combinationPercentage * (movementB.rearLeftDrive - movementA.rearLeftDrive), -1.0, 1.0);
		resultingMovementSettings.frontRightDrive	= Range.clip(movementA.frontRightDrive + combinationPercentage * (movementB.frontRightDrive - movementA.frontRightDrive), -1.0, 1.0);
		resultingMovementSettings.rearRightDrive	= Range.clip(movementA.rearRightDrive + combinationPercentage * (movementB.rearRightDrive - movementA.rearRightDrive), -1.0, 1.0);

		return resultingMovementSettings;
	}

	public static SimpleMovementSettings combine(SimpleMovementSettings movementA, SimpleMovementSettings movementB, double x, double y) {
		double velocity = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
		double combinationPercentage = (90 - Math.toDegrees(Math.abs(Math.atan(y / x)))) / 90;

		return SimpleMovementSettings.combine(movementA, movementB, combinationPercentage).scale(velocity);
	}



	public SimpleMovementSettings clone() {
		SimpleMovementSettings resultingMovementSettings = new SimpleMovementSettings(frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive);
		return resultingMovementSettings;
	}

	public SimpleMovementSettings scale(double scalePercentage) {
		scalePercentage = Math.abs(scalePercentage);
		SimpleMovementSettings resultingMovementSettings = clone();
		resultingMovementSettings.frontLeftDrive	*= scalePercentage;
		resultingMovementSettings.rearLeftDrive		*= scalePercentage;
		resultingMovementSettings.frontRightDrive	*= scalePercentage;
		resultingMovementSettings.rearRightDrive	*= scalePercentage;
		return resultingMovementSettings;
	}

	public String toString() {
		return String.format("FL: %.2f | RL: %.2f | FR: %.2f | RR: %.2f", frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive);
	}
}