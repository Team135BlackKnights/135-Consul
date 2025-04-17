package frc.robot.utils.servos;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Contains the PWM Port IDs 
 */
public class ServoConstantContainer {
	public static final int 
	leftServoPWMPort = 1,
	rightServoPWMPort = 2;
	public static double 
	leftLowerServoBound = -90, leftUpperServoBound = 90,
	rightLowerServoBound = -180, rightUpperServoBound = 180;
	public class ServoMotorConstants{
		//Free current amps is a random guess

		public static DCMotor SimREVSmartServo = new DCMotor(5, 1.32389775, 1, .4, 0.124666375, 1);
	}
	/**
	 * The mode to run the servo in
	 * INRANGE is traditional servo operation,
	 * CONTINUOUS is running it like a motor
	 */
	public enum SimServoMode {
		INRANGE, CONTINUOUS
	}

	public enum ServoType { REVSmartServo; }

	public enum ServoNames {leftServo, rightServo}
}
