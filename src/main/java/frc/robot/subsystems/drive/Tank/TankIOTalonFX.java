// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class TankIOTalonFX implements TankIO {
	private final GyroIO gyro;
	private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatioLow;
	private static final double KP = DriveConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.overallDriveMotorConstantContainer
			.getD();
	private final TalonFX leftLeader;
	private final TalonFX leftFollower;
	private final TalonFX rightLeader;
	private final TalonFX rightFollower;
	private final StatusSignal<Angle> leftPosition;
	private final StatusSignal<AngularVelocity> leftVelocity;
	private final StatusSignal<Voltage> leftAppliedVolts;
	private final StatusSignal<Current> leftLeaderCurrent;
	private final StatusSignal<Current> leftFollowerCurrent;
	private final StatusSignal<Temperature> leftLeaderTemp;
	private final StatusSignal<Temperature> leftFollowerTemp;
	private final StatusSignal<Angle> rightPosition;
	private final StatusSignal<AngularVelocity> rightVelocity;
	private final StatusSignal<Voltage> rightAppliedVolts;
	private final StatusSignal<Current> rightLeaderCurrent;
	private final StatusSignal<Current> rightFollowerCurrent;
	private final StatusSignal<Temperature> rightLeaderTemp;
	private final StatusSignal<Temperature> rightFollowerTemp;
	private final TalonFXConfiguration config;
	private static final Executor currentExecutor = Executors.newFixedThreadPool(8);

	public TankIOTalonFX(GyroIO gyro) {
		this.gyro = gyro;
		if (DriveConstants.driveCanBus == null) {
			this.leftLeader = new TalonFX(DriveConstants.kFrontLeftDrivePort);
			this.leftFollower = new TalonFX(DriveConstants.kBackLeftDrivePort);
			this.rightLeader = new TalonFX(DriveConstants.kFrontRightDrivePort);
			this.rightFollower = new TalonFX(DriveConstants.kBackRightDrivePort);
		} else {
			this.leftLeader = new TalonFX(DriveConstants.kFrontLeftDrivePort, DriveConstants.driveCanBus);
			this.leftFollower = new TalonFX(DriveConstants.kBackLeftDrivePort, DriveConstants.driveCanBus);
			this.rightLeader = new TalonFX(DriveConstants.kFrontRightDrivePort, DriveConstants.driveCanBus);
			this.rightFollower = new TalonFX(DriveConstants.kBackRightDrivePort, DriveConstants.driveCanBus);
		}

		this.leftPosition = leftLeader.getPosition();
		this.leftVelocity = leftLeader.getVelocity();
		this.leftAppliedVolts = leftLeader.getMotorVoltage();
		this.leftLeaderCurrent = leftLeader.getSupplyCurrent();
		this.leftFollowerCurrent = leftFollower.getSupplyCurrent();
		this.leftLeaderTemp = leftLeader.getDeviceTemp();
		this.leftFollowerTemp = leftFollower.getDeviceTemp();
		this.rightPosition = rightLeader.getPosition();
		this.rightVelocity = rightLeader.getVelocity();
		this.rightAppliedVolts = rightLeader.getMotorVoltage();
		this.rightLeaderCurrent = rightLeader.getSupplyCurrent();
		this.rightFollowerCurrent = rightFollower.getSupplyCurrent();
		this.rightLeaderTemp = rightLeader.getDeviceTemp();
		this.rightFollowerTemp = rightFollower.getDeviceTemp();
		this.config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimit = DriveConstants.kMaxDriveCurrent;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.Inverted = DriveConstants.kFrontLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.Slot0.kP = KP;
		config.Slot0.kD = KD;
		leftLeader.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		leftFollower.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kFrontRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		rightLeader.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		rightFollower.getConfigurator().apply(config);
		leftFollower.setControl(new Follower(leftLeader.getDeviceID(),
				DriveConstants.kBackLeftDriveReversed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
		rightFollower.setControl(new Follower(rightLeader.getDeviceID(),
				DriveConstants.kBackRightDriveReversed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
		BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftPosition,
				rightPosition); // Required for odometry, use faster rate
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftVelocity,
				leftAppliedVolts, leftLeaderCurrent, leftFollowerCurrent,
				leftLeaderTemp, leftFollowerTemp, rightVelocity, rightAppliedVolts,
				rightLeaderCurrent, rightFollowerCurrent, rightLeaderTemp,
				rightFollowerTemp);
		leftLeader.optimizeBusUtilization();
		leftFollower.optimizeBusUtilization();
		rightLeader.optimizeBusUtilization();
		rightFollower.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		gyro.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftAppliedVolts,
				leftLeaderCurrent, leftFollowerCurrent, leftLeaderTemp,
				leftFollowerTemp, rightPosition, rightVelocity, rightAppliedVolts,
				rightLeaderCurrent, rightFollowerCurrent, rightLeaderTemp,
				rightFollowerTemp);
		inputs.leftPositionRad = Units
				.rotationsToRadians(leftPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftVelocityRadPerSec = Units
				.rotationsToRadians(leftVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
		inputs.leftCurrentAmps = new double[] {
				leftLeaderCurrent.getValueAsDouble(),
				leftFollowerCurrent.getValueAsDouble()
		};
		inputs.frontLeftDriveTemp = leftLeaderTemp.getValueAsDouble();
		inputs.backLeftDriveTemp = leftFollowerTemp.getValueAsDouble();
		inputs.rightPositionRad = Units
				.rotationsToRadians(rightPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightVelocityRadPerSec = Units
				.rotationsToRadians(rightVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
		inputs.rightCurrentAmps = new double[] {
				rightLeaderCurrent.getValueAsDouble(),
				rightFollowerCurrent.getValueAsDouble()
		};
		inputs.frontRightDriveTemp = rightLeaderTemp.getValueAsDouble();
		inputs.backRightDriveTemp = rightFollowerTemp.getValueAsDouble();
		inputs.gyroConnected = gyroInputs.connected;
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void reset() {
		gyro.reset();
	}

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		leftLeader.setControl(new VoltageOut(leftVolts));
		rightLeader.setControl(new VoltageOut(rightVolts));
	}

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			synchronized (config) {
				config.CurrentLimits.StatorCurrentLimit = amps;
				leftLeader.getConfigurator().apply(config, .25);
				leftFollower.getConfigurator().apply(config, .25);
				rightLeader.getConfigurator().apply(config, .25);
				rightFollower.getConfigurator().apply(config, .25);
			}
		});
		Logger.recordOutput("Drive/CurrentLimit", amps);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		if (DriveConstants.enablePID) {
			leftLeader.setControl(
					new VelocityVoltage(Units.radiansToRotations(leftRadPerSec * GEAR_RATIO)).withEnableFOC(true)
							.withFeedForward(leftFFVolts).withSlot(0).withOverrideBrakeDurNeutral(false)
							.withLimitForwardMotion(false).withLimitReverseMotion(false));
			rightLeader.setControl(
					new VelocityVoltage(Units.radiansToRotations(rightRadPerSec * GEAR_RATIO)).withEnableFOC(true)
							.withFeedForward(rightFFVolts).withSlot(0).withOverrideBrakeDurNeutral(false)
							.withLimitForwardMotion(false).withLimitReverseMotion(false));
		} else {
			setVoltage(convertRadPerSecondToVoltage(leftRadPerSec),
					convertRadPerSecondToVoltage(rightRadPerSec));
		}
	}

	/**
	 * Converts radians per second into voltage that will achieve that value in a
	 * motor.
	 * Takes the angular velocity of the motor (radPerSec),
	 * divides by the theoretical max angular speed (max linear speed / wheel
	 * radius)
	 * and multiplies by 12 (the theoretical standard voltage)
	 * 
	 * @param radPerSec radians per second of the motor
	 * @return the voltage that should be sent to the motor
	 */
	public double convertRadPerSecondToVoltage(double radPerSec) {
		return 12 * radPerSec * (TrainConstants.kWheelDiameter.get() / 2) / DriveConstants.kMaxSpeedMetersPerSecond;

	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.addAll(gyro.getSelfCheckingHardware());
		hardware.add(new SelfCheckingTalonFX("FrontLeftDrive", leftLeader));
		hardware.add(new SelfCheckingTalonFX("BackLeftDrive", leftFollower));
		hardware.add(new SelfCheckingTalonFX("FrontRightDrive", rightLeader));
		hardware.add(new SelfCheckingTalonFX("BackRightDrive", rightFollower));
		return hardware;
	}
}
