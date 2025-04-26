package frc.robot.utils.servos;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;

public class ServoPackage {
	private Servo servo;
	private ServoSim servoSim;
	private double lowerBound, upperBound;
	private final SimServoMode servoMode;
	private final double maxDegreesPerSec;
	/**
	 * Constructs a Servo Package (pairing of servo and servoSim)
	 * 
	 * @param servoPWMPort           The PWM port id of the servo
	 * @param servoMode              The mode that the servo is in (in range, or
	 *                                  continuous)
	 * @param servoType              The type of servo (only one currently
	 *                                  supported is REVSmartServo)
	 * @param initialPositionDegrees The starting position of the servo, in
	 *                                  degrees
	 * @param dtSeconds              Time delay until the periodic function is
	 *                                  updated
	 * @param lowerBound             The lowest value (in degrees) the servo can
	 *                                  get to in range mode
	 * @param upperBound             The highest value (in degrees) the servo can
	 *                                  get to in range mode
	 */
	public ServoPackage(int servoPWMPort, SimServoMode servoMode,
			ServoType servoType, double initialPositionDegrees, double dtSeconds,
			double lowerBound, double upperBound) {
		this.servoMode = servoMode;
		switch (Constants.currentMode) {
		case REAL:
			servo = new Servo(servoPWMPort);
			break;
		default:
			servoSim = new ServoSim(servoMode, servoType, initialPositionDegrees,
					dtSeconds);
			this.lowerBound = lowerBound;
			this.upperBound = upperBound;
			servoSim.setSimBounds(lowerBound, upperBound);
			break;
		}
		this.maxDegreesPerSec = servoType.maxDegreesPerSec;
	}

	/**
	 * Set the servo to a specified angle. Only works in INRANGE mode. 
	 * 
	 * @param degrees the desired angle (in degrees)
	 */
	public void setServoDegrees(double degrees) {
		if (this.servoMode == SimServoMode.INRANGE) {
			switch (Constants.currentMode) {
			case REAL:
				double percent = degrees / (upperBound - lowerBound);
				servo.setPosition(percent);
				break;
			default:
				servoSim.set(degrees);
				break;
			}
		} else {
			throw new IllegalArgumentException("Cannot set angle in CONTINUOUS mode");
		}

	}

	/**
	 * Sets the servo to a certain percent. If it's continuous, sets a POSITION
	 * from 0 to 1, where 0 is maximum left and 1 is maximum right If it's in
	 * range, set it to a constant PERCENTAGE of the max velocity
	 * 
	 * @param percent the percent to set the servo to (-1 to 1) if the servo is
	 *                   continuous, the position to be set to from (0 to 1.0) if
	 *                   it is in range mode
	 */
	public void setServoPercent(double percent) {
		switch (Constants.currentMode) {
		case REAL:
			servo.set(percent);
			break;
		default:
			servoSim.set(percent);
			break;
		}
	}

	/**
	 * @return the angular position of the servo in degrees
	 */
	public double getServoDegrees() {
		switch (Constants.currentMode) {
		case REAL:
			return lowerBound + servo.getPosition() * (upperBound - lowerBound);
		default:
			return servoSim.getAngularPositionDegrees();
		}
	}

	/**
	 * @return the angular velocity in Degrees per second
	 */
	public double getServoVelocityDegreesPerSec(){
		switch (Constants.currentMode) {
		case REAL:
			return lowerBound + servo.getSpeed() * servoSim.getMaxDegreesPerSec();
		default:
			return servoSim.getAngularVelocityDegreesPerSec();
		}
	}
	
	/**
	 * Sets the servo to a certain velocity in degrees per second. If it's in continuous mode
	 * @param degreesPerSec
	 */
	public void setServoDegreesPerSec(double degreesPerSec) {
		if (this.servoMode == SimServoMode.CONTINUOUS) {
			switch (Constants.currentMode) {
				case REAL:
					servo.setSpeed(degreesPerSec / servoSim.getMaxDegreesPerSec());
					break;
				default:
					servoSim.set(degreesPerSec);
					break;
			}


		} else {
			throw new IllegalArgumentException("Cannot set velocity in INRANGE mode");
		}
	}
	/**
	 * @return the angular velocity in Degrees per second
	 */
	public double getServoPercent(){
		switch (Constants.currentMode) {
		case REAL:
			return servo.get();
		default:
			return servoSim.getAngularPositionDegrees() / (upperBound - lowerBound);
		}
	}

	/**
	 * @return the angular velocity in Degrees per second
	 */
	public double getServoSpeedPercent(){
		switch (Constants.currentMode) {
		case REAL:
			return servo.getSpeed();
		default:
			return servoSim.getAngularVelocityDegreesPerSec() / servoSim.getMaxDegreesPerSec();
		}
	}

	/**
	 * Updates the servoSim in simulation. If the servo is real, do NOTHING.
	 */
	public void updateServoSim() {
		switch (Constants.currentMode) {
		case SIM:
			servoSim.updateServoSim();
			return;
		default:
			//does NOTHING.
			return;
		}
	}
	/**
	 * Returns the lower bound of the servo in degrees. 
	 * If the servo is in continuous mode, this returns -1 for both values
	 * @return
	 */
	public double getLowerServoBound() {
		if (this.servoMode == SimServoMode.CONTINUOUS) {
			return -1;
		}
		return lowerBound;
	}
		/**
	 * Returns the upper bound of the servo in degrees. 
	 * If the servo is in continuous mode, this returns -1 for both values
	 * @return
	 */
	public double getUppertServoBound() {
		if (this.servoMode == SimServoMode.CONTINUOUS) {
			return -1;
		}
		return upperBound;
	}
	/**
	 * Gets the servo mode of the servo
	 * @return the servo mode
	 */
	public SimServoMode getSimServoMode(){
		return servoMode;
	}
	/**
	 * Gets the theoretical max speed in degrees per second of the servo
	 * @return the max speed in degrees per second
	 */
    public double getServoMaxVelocityDegreesPerSec() {
       return maxDegreesPerSec;
        }
	
}
