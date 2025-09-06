package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Class designed to hold the values for Kp, Ks, Kv for state-space models and P
 * and D for PID loops. Remember to check whether your loop should be position
 * or velocity
 * 
 * 
 */
public class MotorConstantContainer {
	private double[] valueHolderArray = new double[6];

	/**
	 * Create our SysID constant holder, and alert to any possible SysID
	 * mistakes.
	 * 
	 * @param Ks The Ks value from StaticCharacterization, used in state-space
	 * @param Kv The Kv value from FeedForwardCharacterization, used in state-space.
	 * @param Ka The Ka value from SysID, used in state-space. ALMOST ALWAYS
	 *           UNUSED (0.001)
	 * @param P  The P value, used in PID loops
	 * @param I  The I value from a given constant, used in PID loops
	 * @param D  the D Value, used in PID loops
	 * @see StaticCharacterization
	 * @see FeedForwardCharacterization
	 */
	public MotorConstantContainer(double Ks, double Kv, double Ka, double P,
			double I, double D) {
		if ((Ka <= 0) || (Kv <= 0)) {
			throw new ArithmeticException("Ka and Kv must be greater than 0. kA is " + Ka + " and kV is " + Kv );
		} else if ((P < 0) || (D < 0) || (I < 0)) {
			throw new ArithmeticException(
					"P, I, and D must be greater than or equal to 0");
		} else {
			this.valueHolderArray[0] = Ks;
			this.valueHolderArray[1] = Kv;
			this.valueHolderArray[2] = Ka;
			this.valueHolderArray[3] = P;
			this.valueHolderArray[4] = I;
			this.valueHolderArray[5] = D;
		}
	}

	/**
	 * Easy to use FeedForward implementation
	 * 
	 * @return SimpleMotorFeedforward of Ks and Kv
	 */
	public SimpleMotorFeedforward getFeedforward() {
		return new SimpleMotorFeedforward((getKs()), getKv());
	}

	/**
	 * Easy to use PD implementation
	 * 
	 * @return PID of P and D.
	 */
	public PIDController getPidController() {
		return new PIDController(getP(), getI(), getD());
	}

	/**
	 * @return Ks SysID constant
	 */
	public double getKs() {
		return valueHolderArray[0];
	}

	/**
	 * @return Kv SysID constant
	 */
	public double getKv() {
		return valueHolderArray[1];
	}

	/**
	 * @return Ka SysID constant
	 */
	public double getKa() {
		return valueHolderArray[2];
	}

	/**
	 * @return PID P constant
	 */
	public double getP() {
		return valueHolderArray[3];
	}

	/**
	 * @return PID I constant IF using a position controller
	 */
	public double getI() {
		return valueHolderArray[4];
	}

	/**
	 * @return PID D constant IF using a position controller
	 */
	public double getD() {
		return valueHolderArray[5];
	}
}
