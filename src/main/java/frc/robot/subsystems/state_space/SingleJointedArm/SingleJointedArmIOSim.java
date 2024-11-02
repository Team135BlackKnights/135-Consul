package frc.robot.subsystems.state_space.SingleJointedArm;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.state_space.StateSpaceConstants;

public class SingleJointedArmIOSim implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	/**
	 * Constructs a single-jointed SingleJointedarm simulation with the given
	 * parameters.
	 * 
	 * @param SingleJointedarmGearing The gearing ratio of the SingleJointedarm.
	 *                                   >1 means reduction.
	 * @param SingleJointedarmLength  The length of the SingleJointedarm (in
	 *                                   meters)
	 * @param minPosition             The minimum allowed position of the
	 *                                   SingleJointedarm (in radians).
	 * @param maxPosition             The maximum allowed position of the
	 *                                   SingleJointedarm (in radians).
	 * @param startingPosition        The initial position of the
	 *                                   SingleJointedarm (in radians).
	 *                                   Calculates the position, velocity, and
	 *                                   acceleration of the SingleJointedarm
	 *                                   based on the applied motor voltage and
	 *                                   time step. Provides current position,
	 *                                   velocity, and voltage outputs for
	 *                                   monitoring and control purposes.
	 */
	private final SingleJointedArmSim simArm;

	public SingleJointedArmIOSim() {
		simArm= new SingleJointedArmSim(
			SingleJointedArmS.m_SingleJointedArmPlant, DCMotor.getNEO(1),
			StateSpaceConstants.SingleJointedArm.armGearing,
			StateSpaceConstants.SingleJointedArm.armLength,
			StateSpaceConstants.SingleJointedArm.startingPosition,
			StateSpaceConstants.SingleJointedArm.maxPosition, false,
			StateSpaceConstants.SingleJointedArm.startingPosition,
			StateSpaceConstants.SingleJointedArm.m_KalmanEncoderPosition, StateSpaceConstants.SingleJointedArm.m_KalmanEncoderVelocity);
	}

	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		simArm.setInputVoltage(appliedVolts);
		simArm.update(.02);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = simArm.getAngleRads();
		inputs.velocityRadPerSec = simArm.getVelocityRadPerSec();
		inputs.currentAmps = new double[] {
				MathUtil.clamp(simArm.getCurrentDrawAmps(),
						-StateSpaceConstants.SingleJointedArm.currentLimit,
						StateSpaceConstants.SingleJointedArm.currentLimit)
		}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	/** Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop() {
		setVoltage(0);
	}
}