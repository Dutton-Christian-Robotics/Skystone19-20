package org.firstinspires.ftc.teamcode;

/**
	These constants for field measurement were starting to get used in multiple places. I figured that maybe
	putting them in one location for access by everyone would make sense.
 */

class Measurements {

	// Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
	// We will define some constants and conversions here
	public static final float mmPerInch        = 25.4f;
	public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

	// Constant for Stone Target
	public static final float stoneZ = 2.00f * mmPerInch;

	// Constants for the center support targets
	public static final float bridgeZ = 6.42f * mmPerInch;
	public static final float bridgeY = 23 * mmPerInch;
	public static final float bridgeX = 5.18f * mmPerInch;
	public static final float bridgeRotY = 59;                                 // Units are degrees
	public static final float bridgeRotZ = 180;

	// Constants for perimeter targets
	public static final float halfField = 72 * mmPerInch;
	public static final float quadField  = 36 * mmPerInch;

}
