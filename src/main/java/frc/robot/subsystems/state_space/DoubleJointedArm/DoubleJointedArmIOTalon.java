package frc.robot.subsystems.state_space.DoubleJointedArm;

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

public class DoubleJointedArmIOTalon implements DoubleJointedArmIO {
	private double armVolts = 0.0;
	private double elbowVolts = 0.0;
	private TalonFX arm;
	private TalonFX elbow;
	private final StatusSignal<Angle> armPosition = arm.getPosition();
	private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity();
	private final StatusSignal<Voltage> armAppliedVolts = arm.getMotorVoltage();
	private final StatusSignal<Current> armCurrent = arm.getSupplyCurrent();
	private final StatusSignal<Temperature> armTemp = arm.getDeviceTemp();
	private final StatusSignal<Angle> elbowPosition = elbow.getPosition();
	private final StatusSignal<AngularVelocity> elbowVelocity = elbow.getVelocity();
	private final StatusSignal<Voltage> elbowAppliedVolts = elbow
			.getMotorVoltage();
	private final StatusSignal<Current> elbowCurrent = elbow.getSupplyCurrent();
	private final StatusSignal<Temperature> elbowTemp = elbow.getDeviceTemp();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(2);
	private final TalonFXConfiguration config = new TalonFXConfiguration();

	public DoubleJointedArmIOTalon() {
		arm = new TalonFX(StateSpaceConstants.DoubleJointedArm.kArmMotorID);
		elbow = new TalonFX(StateSpaceConstants.DoubleJointedArm.kElbowMotorID);
		var config = new TalonFXConfiguration();
		config.CurrentLimits.StatorCurrentLimit = StateSpaceConstants.DoubleJointedArm.armCurrentLimit;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.MotorOutput.NeutralMode = StateSpaceConstants.DoubleJointedArm.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.DoubleJointedArm.armInverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		arm.getConfigurator().apply(config);
		config.CurrentLimits.StatorCurrentLimit = StateSpaceConstants.DoubleJointedArm.elbowCurrentLimit;
		config.MotorOutput.NeutralMode = StateSpaceConstants.DoubleJointedArm.isBrake
				? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		config.MotorOutput.Inverted = StateSpaceConstants.DoubleJointedArm.elbowInverted
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		elbow.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, armPosition, armVelocity,
				armAppliedVolts, armCurrent, armTemp, elbowPosition, elbowVelocity,
				elbowAppliedVolts, elbowCurrent, elbowTemp);
		arm.optimizeBusUtilization();
		elbow.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(DoubleJointedArmIOInputs inputs) {
		BaseStatusSignal.refreshAll(armPosition, armVelocity, armAppliedVolts,
				armCurrent, armTemp, elbowPosition, elbowVelocity,
				elbowAppliedVolts, elbowCurrent, elbowTemp);
		arm.setVoltage(armVolts);
		inputs.appliedArmVolts = armAppliedVolts.getValueAsDouble();
		inputs.positionArmRads = Units.rotationsToRadians(BaseStatusSignal
				.getLatencyCompensatedValue(armPosition, armVelocity).magnitude()
				* StateSpaceConstants.DoubleJointedArm.armGearing);
		inputs.velocityArmRadsPerSec = Units
				.rotationsToRadians(armVelocity.getValueAsDouble()
						* StateSpaceConstants.DoubleJointedArm.armGearing);
		inputs.armTemp = armTemp.getValueAsDouble();
		elbow.setVoltage(elbowVolts);
		inputs.appliedElbowVolts = elbowAppliedVolts.getValueAsDouble();
		inputs.positionElbowRads = Units.rotationsToRadians(BaseStatusSignal
				.getLatencyCompensatedValue(elbowPosition, elbowVelocity).magnitude()
				* StateSpaceConstants.DoubleJointedArm.elbowGearing);
		inputs.velocityElbowRadsPerSec = Units
				.rotationsToRadians(elbowVelocity.getValueAsDouble()
						* StateSpaceConstants.DoubleJointedArm.elbowGearing);
		inputs.elbowTemp = elbowTemp.getValueAsDouble();
		inputs.currentAmps = new double[] { armCurrent.getValueAsDouble(),
				elbowCurrent.getValueAsDouble()
		};
	}

	@Override
	public void setVoltage(List<Double> volts) {
		armVolts = volts.get(0);
		elbowVolts = volts.get(1);
	}

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			synchronized (config) {
				config.CurrentLimits.StatorCurrentLimit = amps;
				arm.getConfigurator().apply(config, .25);
				elbow.getConfigurator().apply(config, .25);
			}
		});
	}

	@Override
	/** Stop the arm by telling it to go to 0 arm. */
	public void stop() {
		armVolts = 0;
		elbowVolts = 0;
		arm.stopMotor();
		elbow.stopMotor();
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX("DoubleArmMotor", arm));
		hardware.add(new SelfCheckingTalonFX("DoubleElbowMotor", elbow));
		return hardware;
	}
}
