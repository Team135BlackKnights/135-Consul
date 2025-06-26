package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ModuleIOSparkBase implements ModuleIO {
	// Hardware
	private final SparkBase driveSpark;
	private final SparkBase turnSpark;
	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnRelativeEncoder;
	private final SparkAnalogSensor turnAbsoluteEncoder;
	// Configs
	private SparkBaseConfig driveConfig;
	private SparkBaseConfig turnConfig;
	private EncoderConfig driveEncoderConfig;
	private EncoderConfig turnEncoderConfig;
	private SignalsConfig driveSignalsConfig;
	private SignalsConfig turnSignalsConfig;
	private ClosedLoopConfig driveClosedLoopConfig;
	private MAXMotionConfig driveMaxMotionConfig;
	private ClosedLoopConfig turnClosedLoopConfig;
	private AnalogSensorConfig turnAbsoluteEncoderConfig;
	// Queues
	private final Queue<Double> drivePositionQueue;
	private final Queue<Double> turnPositionQueue;
	private final Rotation2d absoluteEncoderOffset;
	private final Supplier<Rotation2d> absoluteEncoderValue;
	private final String driveName;
	private final String turnName;
	private final boolean isTurnMotorInverted;
	private final boolean isDriveMotorInverted;
	private final boolean isTurnAbsInverted;
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(8);

	public ModuleIOSparkBase(int index) {
		// Init motor & encoder objects
		switch (index) {
			case 0:
				if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
					driveSpark = new SparkMax(DriveConstants.kFrontLeftDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkMax(DriveConstants.kFrontLeftTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkMaxConfig();
					turnConfig = new SparkMaxConfig();
				} else {
					driveSpark = new SparkFlex(DriveConstants.kFrontLeftDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkFlex(DriveConstants.kFrontLeftTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkFlexConfig();
					turnConfig = new SparkFlexConfig();
				}
				driveName = "FrontLeftDrive";
				turnName = "FrontLeftTurn";
				turnAbsoluteEncoder = turnSpark.getAnalog();
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kFrontLeftAbsEncoderOffsetRad);
				isTurnAbsInverted = DriveConstants.kFrontLeftAbsEncoderReversed;
				isDriveMotorInverted = DriveConstants.kFrontLeftDriveReversed;
				isTurnMotorInverted = DriveConstants.kFrontLeftTurningReversed;
				break;
			case 1:
				if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
					driveSpark = new SparkMax(DriveConstants.kFrontRightDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkMax(DriveConstants.kFrontRightTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkMaxConfig();
					turnConfig = new SparkMaxConfig();
				} else {
					driveSpark = new SparkFlex(DriveConstants.kFrontRightDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkFlex(DriveConstants.kFrontRightTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkFlexConfig();
					turnConfig = new SparkFlexConfig();
				}
				driveName = "FrontRightDrive";
				turnName = "FrontRightTurn";
				turnAbsoluteEncoder = turnSpark.getAnalog();
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kFrontRightAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kFrontRightDriveReversed;
				isTurnMotorInverted = DriveConstants.kFrontRightTurningReversed;
				isTurnAbsInverted = DriveConstants.kFrontRightAbsEncoderReversed;
				break;
			case 2:
				if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
					driveSpark = new SparkMax(DriveConstants.kBackLeftDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkMax(DriveConstants.kBackLeftTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkMaxConfig();
					turnConfig = new SparkMaxConfig();
				} else {
					driveSpark = new SparkFlex(DriveConstants.kBackLeftDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkFlex(DriveConstants.kBackLeftTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkFlexConfig();
					turnConfig = new SparkFlexConfig();
				}
				driveName = "BackLeftDrive";
				turnName = "BackLeftTurn";
				turnAbsoluteEncoder = turnSpark.getAnalog();
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kBackLeftAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kBackLeftDriveReversed;
				isTurnMotorInverted = DriveConstants.kBackLeftTurningReversed;
				isTurnAbsInverted = DriveConstants.kBackLeftAbsEncoderReversed;
				break;
			case 3:
				if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
					driveSpark = new SparkMax(DriveConstants.kBackRightDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkMax(DriveConstants.kBackRightTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkMaxConfig();
					turnConfig = new SparkMaxConfig();
				} else {
					driveSpark = new SparkFlex(DriveConstants.kBackRightDrivePort,
							MotorType.kBrushless);
					turnSpark = new SparkFlex(DriveConstants.kBackRightTurningPort,
							MotorType.kBrushless);
					driveConfig = new SparkFlexConfig();
					turnConfig = new SparkFlexConfig();
				}
				driveName = "BackRightDrive";
				turnName = "BackRightTurn";
				turnAbsoluteEncoder = turnSpark.getAnalog();
				absoluteEncoderOffset = new Rotation2d(
						DriveConstants.kBackRightAbsEncoderOffsetRad);
				isDriveMotorInverted = DriveConstants.kBackRightDriveReversed;
				isTurnMotorInverted = DriveConstants.kBackRightTurningReversed;
				isTurnAbsInverted = DriveConstants.kBackRightAbsEncoderReversed;
				break;
			default:
				throw new RuntimeException("Invalid module index");
		}
		driveEncoder = driveSpark.getEncoder();
		turnRelativeEncoder = turnSpark.getEncoder();
		driveSpark.setCANTimeout(250);
		turnSpark.setCANTimeout(250);
		driveConfig = driveConfig.voltageCompensation(12.0);
		turnConfig = turnConfig.voltageCompensation(12.0);
		driveConfig = driveConfig.inverted(isDriveMotorInverted);
		turnConfig = turnConfig.inverted(isTurnMotorInverted);
		driveConfig = driveConfig.smartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		driveConfig = turnConfig.smartCurrentLimit(DriveConstants.kMaxTurnCurrent);
		driveEncoder.setPosition(0.0);
		double driveVelocityConversionFactor = Math.PI / 30 / DriveConstants.TrainConstants.kDriveMotorGearRatioLow;
		double turnVelocityConversionFactor = Math.PI / 30 / DriveConstants.TrainConstants.kTurningMotorGearRatio;
		double drivePositionConversionFactor = 1 / DriveConstants.TrainConstants.kDriveMotorGearRatioLow
				* (2 * Math.PI);
		double turnPositionConversionFactor = 1 / DriveConstants.TrainConstants.kTurningMotorGearRatio * (2 * Math.PI);
		driveEncoderConfig = new EncoderConfig().quadratureAverageDepth(2).quadratureMeasurementPeriod(10)
				.uvwAverageDepth(2).uvwMeasurementPeriod(10).velocityConversionFactor(driveVelocityConversionFactor)
				.positionConversionFactor(drivePositionConversionFactor);
		driveConfig = driveConfig.apply(driveEncoderConfig);
		driveSignalsConfig = new SignalsConfig()
				.primaryEncoderPositionPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.primaryEncoderVelocityPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true)
				.appliedOutputPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.busVoltagePeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz));
		driveConfig = driveConfig.apply(driveSignalsConfig);
		driveMaxMotionConfig = new MAXMotionConfig()
				.maxVelocity(DriveConstants.kMaxSpeedMetersPerSecond
						/ DriveConstants.TrainConstants.kWheelDiameter.get() / 2)
				.maxAcceleration(DriveConstants.maxTranslationalAcceleration.get()
						/ DriveConstants.TrainConstants.kWheelDiameter.get() / 2);
		driveClosedLoopConfig = new ClosedLoopConfig().pidf(
				DriveConstants.overallDriveMotorConstantContainer.getP(),
				DriveConstants.overallDriveMotorConstantContainer.getI(),
				DriveConstants.overallDriveMotorConstantContainer.getD(),
				DriveConstants.overallDriveMotorConstantContainer.getKv())
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(driveMaxMotionConfig);
		driveConfig = driveConfig.apply(driveClosedLoopConfig);
		// Drive Config Done!
		turnEncoderConfig = new EncoderConfig().quadratureAverageDepth(2).quadratureMeasurementPeriod(10)
				.uvwAverageDepth(2).uvwMeasurementPeriod(10)
				.velocityConversionFactor(turnVelocityConversionFactor)
				.positionConversionFactor(turnPositionConversionFactor);
		turnConfig = turnConfig.apply(turnEncoderConfig);
		//3.3 is the voltage here
		turnAbsoluteEncoderConfig = new AnalogSensorConfig().inverted(isTurnAbsInverted)
				.positionConversionFactor(1 / RobotController.getVoltage3V3() * 2 * Math.PI)
				.velocityConversionFactor(1 / RobotController.getVoltage3V3() * 2 * Math.PI);
		turnConfig = turnConfig.apply(turnAbsoluteEncoderConfig);
		turnSignalsConfig = new SignalsConfig().analogPositionAlwaysOn(true).analogVelocityAlwaysOn(true)
				.analogPositionPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.analogVelocityPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.primaryEncoderPositionPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.primaryEncoderPositionAlwaysOn(true)
				.appliedOutputPeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz))
				.busVoltagePeriodMs((int) (1000 / DriveConstants.TrainConstants.odomHz));
		turnConfig = turnConfig.apply(turnSignalsConfig);
		turnClosedLoopConfig = new ClosedLoopConfig().pidf(
				DriveConstants.overallTurningMotorConstantContainer.getP(),
				DriveConstants.overallTurningMotorConstantContainer.getI(),
				DriveConstants.overallTurningMotorConstantContainer.getD(),
				DriveConstants.overallTurningMotorConstantContainer.getKv())
				.feedbackSensor(FeedbackSensor.kAnalogSensor).positionWrappingEnabled(true)
				.positionWrappingInputRange(-Math.PI, Math.PI);
		turnConfig = turnConfig.apply(turnClosedLoopConfig);
		driveSpark.setCANTimeout(0);
		turnSpark.setCANTimeout(0);
		absoluteEncoderValue = () -> Rotation2d
				.fromRadians(turnAbsoluteEncoder.getPosition() - absoluteEncoderOffset.getRadians());
		drivePositionQueue = OdometryThread.registerInput(driveEncoder::getPosition);
		turnPositionQueue = OdometryThread.registerInput(() -> absoluteEncoderValue.get().getRadians());
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			driveMaxMotionConfig = driveMaxMotionConfig
					.maxAcceleration(DriveConstants.maxTranslationalAcceleration.get()
							/ DriveConstants.TrainConstants.kWheelDiameter.get() / 2);
			driveClosedLoopConfig = driveClosedLoopConfig.apply(driveMaxMotionConfig);
			driveConfig = driveConfig.apply(driveClosedLoopConfig);
		}, DriveConstants.maxTranslationalAcceleration);
		inputs.drivePositionRads = driveEncoder.getPosition();
		inputs.driveVelocityRadsPerSec = driveEncoder.getVelocity();
		inputs.driveAppliedVolts = driveSpark.getAppliedOutput()
				* driveSpark.getBusVoltage();
		inputs.driveSupplyCurrentAmps = driveSpark.getOutputCurrent();
		inputs.driveMotorTemp = driveSpark.getMotorTemperature();
		inputs.turnAbsolutePosition = absoluteEncoderValue.get();
		inputs.turnPosition = Rotation2d.fromRadians(turnRelativeEncoder.getPosition());
		inputs.turnVelocityRadsPerSec = turnAbsoluteEncoder.getVelocity();
		inputs.turnAppliedVolts = turnSpark.getAppliedOutput()
				* turnSpark.getBusVoltage();
		inputs.turnSupplyCurrentAmps = turnSpark.getOutputCurrent();
		inputs.turnMotorTemp = turnSpark.getMotorTemperature();
		inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
				.mapToDouble(motorPositionRevs -> motorPositionRevs
						* (DriveConstants.TrainConstants.kWheelDiameter.get() / 2))
				.toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream()
				.map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void runDriveVolts(double volts) {
		driveSpark.setVoltage(volts);
	}

	@Override
	public void runTurnVolts(double volts) {
		turnSpark.setVoltage(volts);
	}

	@Override
	public void runCharacterization(double input) {
		runDriveVolts(input);
	}

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		driveSpark.getClosedLoopController().setReference(velocityRadsPerSec, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0,
				feedForward);
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		double expectedAngle = angleRads - absoluteEncoderOffset.getRadians();
		turnSpark.getClosedLoopController().setReference(expectedAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
	}

	@Override
	public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
		driveClosedLoopConfig = driveClosedLoopConfig.pidf(kP, kI, kD, kV);
		driveConfig = driveConfig.apply(driveClosedLoopConfig);
		updateExecutor(false);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD, double kS, double kV, double deadbandAmps) {
		turnClosedLoopConfig = turnClosedLoopConfig.pidf(kP, kI, kD, kV);
		turnConfig = turnConfig.apply(turnClosedLoopConfig);
		updateExecutor(true);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveConfig = driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
		updateExecutor(false);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnConfig = turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
		updateExecutor(true);
	}

	@Override
	public void stop() {
		driveSpark.stopMotor();
		turnSpark.stopMotor();
	}

	private void updateExecutor(boolean isTurn) {
		if (isTurn) {
			currentExecutor.execute(() -> {
				REVLibError setError = turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters,
						PersistMode.kNoPersistParameters);
				if (setError != REVLibError.kOk) {
					Logger.recordOutput("Drive/updateCode_" + turnName, setError.name());
				} else {
					Logger.recordOutput("Drive/updateCode_" + turnName, "OK");
				}
			});
		} else {
			currentExecutor.execute(() -> {
				REVLibError setError = driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters,
						PersistMode.kNoPersistParameters);
				if (setError != REVLibError.kOk) {
					Logger.recordOutput("Drive/updateCode_" + driveName, setError.name());
				} else {
					Logger.recordOutput("Drive/updateCode_" + driveName, "OK");
				}
			});
		}
	}

	@Override
	public void setCurrentLimit(int amps) {
		driveConfig = driveConfig.smartCurrentLimit(amps);
		updateExecutor(false);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase(driveName, driveSpark));
		hardware.add(new SelfCheckingSparkBase(turnName, turnSpark));
		return hardware;
	}
}
