package frc.robot.subsystems.state_space.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorIOSim implements ElevatorIO {
	private double appliedVolts = 0.0;
	/**
	 * Constructs a single-jointed Elevator simulation with the given parameters.
	 * 
	 * @param ElevatorGearing  The gearing ratio of the Elevator. >1 means
	 *                            reduction.
	 * @param ElevatorLength   The length of the Elevator (in meters)
	 * @param minPosition      The minimum allowed position of the Elevator (in
	 *                            radians).
	 * @param maxPosition      The maximum allowed position of the Elevator (in
	 *                            radians).
	 * @param startingPosition The initial position of the Elevator (in radians).
	 *                            Calculates the position, velocity, and
	 *                            acceleration of the Elevator based on the
	 *                            applied motor voltage and time step. Provides
	 *                            current position, velocity, and voltage outputs
	 *                            for monitoring and control purposes.
	 */
	private ElevatorSim simElevator = new ElevatorSim(ElevatorS.m_elevatorPlant,
			DCMotor.getNEO(1), StateSpaceConstants.Elevator.startingPosition,
			StateSpaceConstants.Elevator.maxPosition, false,
			StateSpaceConstants.Elevator.startingPosition);

	public ElevatorIOSim() {}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		simElevator.setInputVoltage(appliedVolts);
		simElevator.update(.02);
		inputs.appliedVolts = appliedVolts;
		inputs.positionMeters = simElevator.getPositionMeters();
		inputs.velocityMetersPerSec = simElevator.getVelocityMetersPerSecond();
		inputs.currentAmps = new double[] {
				MathUtil.clamp(simElevator.getCurrentDrawAmps(),
						-StateSpaceConstants.Elevator.currentLimit,
						StateSpaceConstants.Elevator.currentLimit)
		}; //Coconut, it somehow pulls "200" amps at full.. Just NO.
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() { setVoltage(0); }
}