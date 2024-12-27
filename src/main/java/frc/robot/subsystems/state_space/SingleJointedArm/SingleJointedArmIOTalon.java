package frc.robot.subsystems.state_space.SingleJointedArm;

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

public class SingleJointedArmIOTalon implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private TalonFX arm;
	private final StatusSignal<Angle> armPosition = arm.getPosition();
	private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity();
	private final StatusSignal<Voltage> armAppliedVolts = arm.getMotorVoltage();
	private final StatusSignal<Current> armCurrent = arm.getSupplyCurrent();
	private final StatusSignal<Temperature> armTemp = arm.getDeviceTemp();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(1);
	private final TalonFXConfiguration config = new TalonFXConfiguration();

	public SingleJointedArmIOTalon() {
		arm = new TalonFX(StateSpaceConstants.SingleJointedArm.kMotorID);
		config.CurrentLimits.StatorCurrentLimit = StateSpaceConstants.SingleJointedArm.currentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.SingleJointedArm.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.SingleJointedArm.inverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		arm.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, armPosition, armVelocity,
				armAppliedVolts, armCurrent, armTemp);
		arm.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts,
				armCurrent, armTemp);
		arm.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.armTemp = armTemp.getValueAsDouble();
		inputs.positionRad = Units.rotationsToRadians(BaseStatusSignal
				.getLatencyCompensatedValue(armPosition, armVelocity, .2).magnitude()
				* StateSpaceConstants.SingleJointedArm.armGearing);;
		inputs.velocityRadPerSec = Units.rotationsToRadians(armVelocity.getValueAsDouble()
				* StateSpaceConstants.SingleJointedArm.armGearing);
		inputs.currentAmps = new double[] { armCurrent.getValueAsDouble()
		};
	}

	@Override
	public void setVoltage(double volts) { appliedVolts = volts; }

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			synchronized (config) {
				config.CurrentLimits.StatorCurrentLimit = amps;
				arm.getConfigurator().apply(config, .25);
			}
		});
	}

	@Override
	/** Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop() {
		setVoltage(0);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("SingleArm", arm));
		return hardware;
	}
}