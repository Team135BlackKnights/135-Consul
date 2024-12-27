package frc.robot.subsystems.state_space.Elevator;

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

public class ElevatorIOTalon implements ElevatorIO {
	private double appliedVolts = 0.0;
	private TalonFX elevator;
	private final StatusSignal<Angle> elevatorPosition = elevator.getPosition();
	private final StatusSignal<AngularVelocity> elevatorVelocity = elevator.getVelocity();
	private final StatusSignal<Voltage> elevatorAppliedVolts = elevator
			.getMotorVoltage();
	private final StatusSignal<Current> elevatorCurrent = elevator
			.getSupplyCurrent();
	private final StatusSignal<Temperature> elevatorTemp = elevator.getDeviceTemp();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(1);
	private final TalonFXConfiguration config = new TalonFXConfiguration();

	public ElevatorIOTalon() {
		elevator = new TalonFX(StateSpaceConstants.Elevator.kMotorID);
		config.CurrentLimits.StatorCurrentLimit = StateSpaceConstants.Elevator.currentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.Elevator.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.Elevator.inverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		elevator.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, elevatorPosition,
				elevatorVelocity, elevatorAppliedVolts, elevatorCurrent,
				elevatorTemp);
		elevator.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity,
				elevatorAppliedVolts, elevatorCurrent, elevatorTemp);
		elevator.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.elevatorTemp = elevatorTemp.getValueAsDouble();
		inputs.positionMeters = Units.rotationsToRadians(BaseStatusSignal
				.getLatencyCompensatedValue(elevatorPosition, elevatorVelocity, .2).magnitude()
				* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.velocityMetersPerSec = Units
				.rotationsToRadians(elevatorVelocity.getValueAsDouble()
						* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.currentAmps = new double[] { elevatorCurrent.getValueAsDouble()
		};
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			synchronized (config) {
				config.CurrentLimits.StatorCurrentLimit = amps;
				elevator.getConfigurator().apply(config, .25);
			}
		});
	}

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() { setVoltage(0); }

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("Elevator", elevator));
		return hardware;
	}
}