package frc.robot.utils.servos;

import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;

/**
 * Class to handle simulation of a servo
 */
public class ServoSim {
	private double[] bounds = { 0, 0
	};
	private double positionDegrees = 0, velocityDegreesPerSecond = 0,
			maxDegreesPerSecond = 0, setpoint = 0, deltaTheta = 0, deadband = 0,
			simDtSeconds;
	SimServoMode simMode;

	/**
	 * Creates a new servo simulation
	 * 
	 * @param runMode                How the servo is running (continuously or in
	 *                                  range)
	 * @param type                   the type of servo (REVSmartServo is the only
	 *                                  supported one currently)
	 * @param initialPositionDegrees starting position of the servo
	 * @param dtSeconds              Length of time between periodics
	 */
	public ServoSim(SimServoMode runMode, ServoType type,
			double initialPositionDegrees, double dtSeconds) {
		simMode = runMode;
		positionDegrees = initialPositionDegrees;
		maxDegreesPerSecond = type.maxDegreesPerSec;
		simDtSeconds = dtSeconds;
		deadband = maxDegreesPerSecond * simDtSeconds;
		//Momentum is obnoxiously low, shaft doesnt have that much inertia
	}

	/**
	 * Set the bounds of a simulated servo in Degrees. This must be called
	 * 
	 * @param leftBound  The leftmost position the servo can go to (in Degrees)
	 * @param rightBound The rightmost position the servo can go to (in Degrees)
	 */
	public void setSimBounds(double leftBound, double rightBound) {
		bounds = new double[] { leftBound, rightBound
		};
		deltaTheta = rightBound - leftBound;
		//Assumes servo starts at leftBound
		positionDegrees = leftBound;
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
	public void set(double percent) {
		switch (simMode) {
		case CONTINUOUS:
			velocityDegreesPerSecond = maxDegreesPerSecond * percent;
			break;
		default:
			if (percent < 0) {
				throw new ArithmeticException(
						"Percent must be greater than zero for case INRANGE");
			}
			setpoint = bounds[0] + deltaTheta * percent; //-90 + 180*.02
			if (setpoint <= bounds[0]) {
				setpoint = bounds[0];
			}
			else if (setpoint >= bounds[1]) {
				setpoint = bounds[1];
			}
			break;
		}
	}

	/**
	 * Sets the angle of the servo. ONLY WORKS IN INRANGE MODE, will do nothing
	 * in continuous mode
	 * 
	 * @param degrees
	 */
	public void setAngle(double degrees) {
		switch (simMode) {
		case INRANGE:
			if (setpoint <= bounds[0]) {
				setpoint = bounds[0];
			}
			else if (setpoint >= bounds[1]) {
				setpoint = bounds[1];
			} else {
				setpoint = degrees;
			}
			break;
		default:
			System.err.println("SetAngle should not be called in continuous!");
			//does nothing here
			break;
		}
	}

	/**
	 * @return the angular velocity in Degrees per second
	 */
	public double getAngularVelocityDegreesPerSec() {
		return velocityDegreesPerSecond;
	}

	/**
	 * @return the angular position in Degrees
	 */
	public double getAngularPositionDegrees() { return positionDegrees; }

	/**
	 * Call this to update your servo state (typically done in a periodic). Call
	 * this function in the dtSeconds specified in the constructor
	 */
	public void updateServoSim() {
		switch (simMode) {
		//computes where it should be next
		case INRANGE:
			double deltaPos = setpoint - positionDegrees; //95 -90  = 5
			double nextPos = (positionDegrees
					+ maxDegreesPerSecond * simDtSeconds * Math.signum(deltaPos))
					% 360;
			//If its not within bounds, don't set it to nextPos
			if (!(nextPos < bounds[0] && velocityDegreesPerSecond < 0)
					&& !(nextPos > bounds[1] && velocityDegreesPerSecond > 0)) {
				//If nextPos is beyond the setpoint, don't set it to nextPos
				if (!(velocityDegreesPerSecond > 0 && nextPos > setpoint)
						&& !(velocityDegreesPerSecond < 0 && nextPos < setpoint)) {
					//if its in deadband, go to setpoint
					if (Math.abs(deltaPos) < deadband) {
						positionDegrees = setpoint;
					} else {
						positionDegrees = nextPos;
					}
				}
			}
		default:
			//This is just p = p0 + v0t
			positionDegrees += velocityDegreesPerSecond * simDtSeconds;
			positionDegrees %= 360;
			break;
		}
	}
	public double getMaxDegreesPerSec(){
		return maxDegreesPerSecond;
	}
}
