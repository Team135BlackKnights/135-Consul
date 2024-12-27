package frc.robot.subsystems.state_space.Flywheel;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelIOSim implements FlywheelIO {
	private double appliedVolts = 0.0;
	// Yes, calling the plant in the IO is bad. I know. but I can't remake it.
	private final FlywheelSim flywheelSim;

	public FlywheelIOSim() {
		flywheelSim = new FlywheelSim(FlywheelS.flywheelPlant,
				DCMotor.getNEO(1),
				.5);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		flywheelSim.setInputVoltage(appliedVolts);
		flywheelSim.update(.02);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = 0.0; // no pos in flywheel
		inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
		inputs.currentAmps = new double[] {
				MathUtil.clamp(flywheelSim.getCurrentDrawAmps(),
						-StateSpaceConstants.Flywheel.currentLimit,
						StateSpaceConstants.Flywheel.currentLimit)
		}; // Coconut, it somehow pulls "200" amps at full.. Just NO.
	}

	@Override
	public void setVoltage(double volts) {
		appliedVolts = volts;
	}

	@Override
	/** Stop the flywheel by telling it to go to 0 rpm. */
	public void stop() {
		appliedVolts = 0;
	}
}
