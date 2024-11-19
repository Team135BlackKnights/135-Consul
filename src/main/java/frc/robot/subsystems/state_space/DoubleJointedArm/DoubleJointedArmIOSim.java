package frc.robot.subsystems.state_space.DoubleJointedArm;

import java.util.List;

public class DoubleJointedArmIOSim implements DoubleJointedArmIO {
	private double armVolts = 0.0;
	private double elbowVolts = 0.0;
	public double expectedArmRads = 0;
	public double expectedElbowRads = 0;

	public DoubleJointedArmIOSim() {}

	@Override
	public void updateInputs(DoubleJointedArmIOInputs inputs) {
		inputs.appliedArmVolts = armVolts;
		inputs.positionArmRads = expectedArmRads;
		inputs.appliedElbowVolts = elbowVolts;
		inputs.positionElbowRads = expectedElbowRads;
		inputs.currentAmps = new double[] { 0, 0 };
	}

	@Override
	public void setVoltage(List<Double> volts) {
		armVolts = volts.get(0);
		elbowVolts = volts.get(1);
	}

	@Override
	public void setExpectedPositions(double armRads, double elbowRads) {
		this.expectedArmRads = armRads;
		this.expectedElbowRads = elbowRads;
	}

	@Override
	/** Stop the arm by telling it to go to 0 arm. */
	public void stop() {
		armVolts = 0;
		elbowVolts = 0;
	}
}
