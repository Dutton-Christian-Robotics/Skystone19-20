package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
	One problem with running motors is that if you mount them backwards, forwards and reverse get
	switched up and it can be mind-numbing trying to remember which way you need to make a motor
	go. To solve that problem, this subclass does one thing: it knows which way is forward for
	itself. By pulling that info from a configuration file, we can set that when we initialize the
	bot and then we just ask the motor to go in the right CONCEPTUAL direction and the motor itself
	will choose way way it should PHYSICALLY move.

	FYI, finding the right way to subclass DcMotor was not entirely obvious.

*/

public class SmartDcMotor extends DcMotorImpl {
 	public DcMotor.Direction forwardDirection;
 	public DcMotor.Direction reverseDirection;

 	public enum MotorType {
	 	HEX,
	 	TORQUENADO
 	}


 	/*
		This is the constructor we call from the bot class. Call super class with information from
		the hardware map.
 	*/
	public SmartDcMotor(HardwareMap hwMap, String motorName, DcMotorSimple.Direction direction) {
		super(hwMap.get(DcMotor.class, motorName).getController(), hwMap.get(DcMotor.class, motorName).getPortNumber());
		establishForwardDirection(direction);

	}

	public SmartDcMotor(HardwareMap hwMap, String motorName, MotorType motorType, DefenderBot.MotorLocation motorLocation) {
		super(hwMap.get(DcMotor.class, motorName).getController(), hwMap.get(DcMotor.class, motorName).getPortNumber());
		establishForwardDirection(motorType, motorLocation);

	}

	/*
		I'm not sure we need these three constructors but I haven't tried removing them yet.
	*/
	public SmartDcMotor(DcMotorController controller, int portNumber) {
		super(controller, portNumber);
	}

	public SmartDcMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction) {
		super(controller, portNumber, direction);
	}

	public SmartDcMotor(DcMotorController controller, int portNumber, DcMotorSimple.Direction direction, MotorConfigurationType motorType) {
		super(controller, portNumber, direction, motorType);
	}

	/*
		In our code, motors are not directly instantiated with a constructor. They're returned
		from a hardware map. This function can be called to establish which way is the right
		way for the motor to spin.
	*/
	public void establishForwardDirection(DcMotor.Direction d) {
		forwardDirection = d;
		if (d == DcMotor.Direction.FORWARD) {
			reverseDirection = DcMotor.Direction.REVERSE;
		} else {
			reverseDirection = DcMotor.Direction.FORWARD;
		}

	}
	public void establishForwardDirection(MotorType motorType, DefenderBot.MotorLocation motorLocation) {
		if (motorType == MotorType.TORQUENADO) {
			if (motorLocation == DefenderBot.MotorLocation.LEFT) {
				forwardDirection = DcMotor.Direction.REVERSE;
				reverseDirection = DcMotor.Direction.FORWARD;
			} else if (motorLocation == DefenderBot.MotorLocation.RIGHT) {
				forwardDirection = DcMotor.Direction.FORWARD;
				reverseDirection = DcMotor.Direction.REVERSE;
			}
		}
	}




	// A convenience function to spin the motor forward CONCEPTUALLY (might not be forward physically)
	public void setDirectionForward() {
		setDirection(forwardDirection);
	}

	// A convenience function to spin the motor in reverse CONCEPTUALLY (might not be reverse physically)
	public void setDirectionReverse() {
		setDirection(reverseDirection);
	}

}