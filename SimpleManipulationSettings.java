package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import java.lang.Math;


/**
 * This class id designed to help facilitate
 *
 * NOTE: The name of this class might change in the future to something more elegant or better suited.
 *
 */

public class SimpleManipulationSettings extends SimpleSettings {

	public double liftMotor = 0;
	public double tiltMotor = 0;
	public double extendMotor = 0;
	public double grabMotor = 0;

	public SimpleManipulationSettings(double lm, double tm, double em, double gm) {
		liftMotor = lm;
		tiltMotor = tm;
		extendMotor = em;
		grabMotor = gm;
	}

	public static SimpleManipulationSettings combine(SimpleManipulationSettings movementA, SimpleManipulationSettings movementB, double combinationPercentage) {
		if (combinationPercentage > 1) {
			combinationPercentage /= 100;
		}
		SimpleManipulationSettings resultingManipulationSettings = new SimpleManipulationSettings(0,0,0,0);

		resultingManipulationSettings.liftMotor	= Range.clip(movementA.liftMotor + combinationPercentage * (movementB.liftMotor - movementA.liftMotor), -1.0, 1.0);
		resultingManipulationSettings.extendMotor		= Range.clip(movementA.extendMotor + combinationPercentage * (movementB.extendMotor - movementA.extendMotor), -1.0, 1.0);
		resultingManipulationSettings.tiltMotor	= Range.clip(movementA.tiltMotor + combinationPercentage * (movementB.tiltMotor - movementA.tiltMotor), -1.0, 1.0);
		resultingManipulationSettings.grabMotor	= Range.clip(movementA.grabMotor + combinationPercentage * (movementB.grabMotor - movementA.grabMotor), -1.0, 1.0);

		return resultingManipulationSettings;
	}

	public static SimpleManipulationSettings combine(SimpleManipulationSettings movementA, SimpleManipulationSettings movementB, double x, double y) {
		double velocity = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
		double combinationPercentage = (90 - Math.toDegrees(Math.abs(Math.atan(y / x)))) / 90;

		return SimpleManipulationSettings.combine(movementA, movementB, combinationPercentage).scale(velocity);
	}



	public SimpleManipulationSettings clone() {
		SimpleManipulationSettings resultingManipulationSettings = new SimpleManipulationSettings(liftMotor, extendMotor, tiltMotor, grabMotor);
		return resultingManipulationSettings;
	}

	public SimpleManipulationSettings scale(double scalePercentage) {
		scalePercentage = Math.abs(scalePercentage);
		SimpleManipulationSettings resultingManipulationSettings = clone();
		resultingManipulationSettings.liftMotor	*= scalePercentage;
		resultingManipulationSettings.extendMotor		*= scalePercentage;
		resultingManipulationSettings.tiltMotor	*= scalePercentage;
		resultingManipulationSettings.grabMotor	*= scalePercentage;
		return resultingManipulationSettings;
	}

	public String toString() {
		return String.format("Lift: %.2f | Tilt: %.2f | Extend: %.2f | Grab: %.2f", liftMotor, tiltMotor, extendMotor, grabMotor);
	}
}