package frc.robot.subsystems.state_space.Flywheel;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelIOTalon implements FlywheelIO {
	private double appliedVolts = 0.0;
	private TalonFX flywheel;
	private final StatusSignal<Angle> flywheelPosition = flywheel.getPosition();
	private final StatusSignal<AngularVelocity> flywheelVelocity = flywheel.getVelocity();
	private final StatusSignal<Voltage> flywheelAppliedVolts = flywheel
			.getMotorVoltage();
	private final StatusSignal<Current> flywheelCurrent = flywheel
			.getSupplyCurrent();
	private final StatusSignal<Temperature> flywheelTemp = flywheel.getDeviceTemp();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(1);
	private final TalonFXConfiguration config = new TalonFXConfiguration();

	public FlywheelIOTalon() {
		flywheel = new TalonFX(StateSpaceConstants.Flywheel.kMotorID);
		config.CurrentLimits.StatorCurrentLimit = StateSpaceConstants.Flywheel.currentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.Flywheel.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.Flywheel.inverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		flywheel.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, flywheelPosition,
				flywheelVelocity, flywheelAppliedVolts, flywheelCurrent,
				flywheelTemp);
		flywheel.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		BaseStatusSignal.refreshAll(flywheelPosition, flywheelVelocity,
				flywheelAppliedVolts, flywheelCurrent, flywheelTemp);
		flywheel.setVoltage(appliedVolts);
		inputs.appliedVolts = flywheelAppliedVolts.getValueAsDouble();
		inputs.positionRad = Units.rotationsToRadians(flywheelPosition.getValueAsDouble()
				* StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.velocityRadPerSec = Units
				.rotationsToRadians(flywheelVelocity.getValueAsDouble()
						* StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.currentAmps = new double[] { flywheelCurrent.getValueAsDouble()
		};
		inputs.flywheelTemp = flywheelTemp.getValueAsDouble();
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			synchronized (config) {
				config.CurrentLimits.StatorCurrentLimit = amps;
				flywheel.getConfigurator().apply(config, .25);
			}
		});
	}

	@Override
	/** Stop the flywheel by telling it to go to 0 rpm. */
	public void stop() {
		appliedVolts = 0;
		flywheel.stopMotor();
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("Flywheel", flywheel));
		return hardware;
	}
}
