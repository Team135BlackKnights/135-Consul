package frc.robot.subsystems.drive.FastSwerve;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
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

public class ModuleIOSparkBase implements ModuleIO {
	// Hardware
	private final CANSparkBase driveSpark;
	private final CANSparkBase turnSpark;
	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turnRelativeEncoder;
	private final SparkAnalogSensor turnAbsoluteEncoder;
	// Controllers
	private final PIDController driveController;
	private final PIDController turnController;
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
	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer
					.getKs(),
			0);

	public ModuleIOSparkBase(int index) {
		// Init motor & encoder objects
		switch (index) {
		case 0:
			if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
				driveSpark = new CANSparkMax(DriveConstants.kFrontLeftDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkMax(DriveConstants.kFrontLeftTurningPort,
						MotorType.kBrushless);
			} else {
				driveSpark = new CANSparkFlex(DriveConstants.kFrontLeftDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkFlex(DriveConstants.kFrontLeftTurningPort,
						MotorType.kBrushless);
			}
			driveName = "FrontLeftDrive";
			turnName = "FrontLeftTurn";
			turnAbsoluteEncoder = turnSpark.getAnalog(Mode.kAbsolute);
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kFrontLeftAbsEncoderOffsetRad);
			isTurnAbsInverted = DriveConstants.kFrontLeftAbsEncoderReversed;
			isDriveMotorInverted = DriveConstants.kFrontLeftDriveReversed;
			isTurnMotorInverted = DriveConstants.kFrontLeftTurningReversed;
			break;
		case 1:
			if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
				driveSpark = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkMax(DriveConstants.kFrontRightTurningPort,
						MotorType.kBrushless);
			} else {
				driveSpark = new CANSparkFlex(DriveConstants.kFrontRightDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkFlex(DriveConstants.kFrontRightTurningPort,
						MotorType.kBrushless);
			}
			driveName = "FrontRightDrive";
			turnName = "FrontRightTurn";
			turnAbsoluteEncoder = turnSpark.getAnalog(Mode.kAbsolute);
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kFrontRightAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kFrontRightDriveReversed;
			isTurnMotorInverted = DriveConstants.kFrontRightTurningReversed;
			isTurnAbsInverted = DriveConstants.kFrontRightAbsEncoderReversed;
			break;
		case 2:
			if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
				driveSpark = new CANSparkMax(DriveConstants.kBackLeftDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkMax(DriveConstants.kBackLeftTurningPort,
						MotorType.kBrushless);
			} else {
				driveSpark = new CANSparkFlex(DriveConstants.kBackLeftDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkFlex(DriveConstants.kBackLeftTurningPort,
						MotorType.kBrushless);
			}
			driveName = "BackLeftDrive";
			turnName = "BackLeftTurn";
			turnAbsoluteEncoder = turnSpark.getAnalog(Mode.kAbsolute);
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kBackLeftAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kBackLeftDriveReversed;
			isTurnMotorInverted = DriveConstants.kBackLeftTurningReversed;
			isTurnAbsInverted = DriveConstants.kBackLeftAbsEncoderReversed;
			break;
		case 3:
			if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
				driveSpark = new CANSparkMax(DriveConstants.kBackRightDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkMax(DriveConstants.kBackRightTurningPort,
						MotorType.kBrushless);
			} else {
				driveSpark = new CANSparkFlex(DriveConstants.kBackRightDrivePort,
						MotorType.kBrushless);
				turnSpark = new CANSparkFlex(DriveConstants.kBackRightTurningPort,
						MotorType.kBrushless);
			}
			driveName = "BackRightDrive";
			turnName = "BackRightTurn";
			turnAbsoluteEncoder = turnSpark.getAnalog(Mode.kAbsolute);
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
		driveSpark.restoreFactoryDefaults();
		turnSpark.restoreFactoryDefaults();
		driveSpark.setCANTimeout(250);
		turnSpark.setCANTimeout(250);
		for (int i = 0; i < 30; i++) {
			turnSpark.setInverted(isTurnMotorInverted);
			driveSpark.setInverted(isDriveMotorInverted);
			driveSpark.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
			turnSpark.setSmartCurrentLimit(DriveConstants.kMaxTurnCurrent);
			driveSpark.enableVoltageCompensation(12.0);
			turnSpark.enableVoltageCompensation(12.0);
			driveEncoder.setPosition(0.0);
			driveEncoder.setMeasurementPeriod(10);
			driveEncoder.setAverageDepth(2);
			turnRelativeEncoder.setPosition(0.0);
			turnRelativeEncoder.setMeasurementPeriod(10);
			turnRelativeEncoder.setAverageDepth(2);
			driveSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
					(int) (1000.0 / 250));
			turnSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus2,
					(int) (1000.0 / 250));
		}
		driveSpark.burnFlash();
		turnSpark.burnFlash();
		//driveSpark.setCANTimeout(0);
		//turnSpark.setCANTimeout(0);
		absoluteEncoderValue = () -> Rotation2d
				.fromRotations(turnAbsoluteEncoder.getVoltage()
						/ RobotController.getVoltage3V3())
				.minus(absoluteEncoderOffset).times(isTurnAbsInverted ? -1 : 1);
		drivePositionQueue = OdometryThread.registerInput(driveEncoder::getPosition);
		turnPositionQueue = OdometryThread.registerInput(() -> absoluteEncoderValue.get().getRadians());
		// Init Controllers
		driveController = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
				.getPidController();
		turnController = DriveConstants.TrainConstants.overallTurningMotorConstantContainer
				.getPidController();
		turnController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRads = Units
				.rotationsToRadians(driveEncoder.getPosition()
						/ DriveConstants.TrainConstants.kDriveMotorGearRatio);
		inputs.driveVelocityRadsPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()
						/ DriveConstants.TrainConstants.kDriveMotorGearRatio);
		inputs.driveAppliedVolts = driveSpark.getAppliedOutput()
				* driveSpark.getBusVoltage();
		inputs.driveSupplyCurrentAmps = driveSpark.getOutputCurrent();
		inputs.driveMotorTemp = driveSpark.getMotorTemperature();
		inputs.turnAbsolutePosition = absoluteEncoderValue.get();
		inputs.turnPosition = Rotation2d.fromRadians(
				Units.rotationsToRadians(turnRelativeEncoder.getPosition()
						/ DriveConstants.TrainConstants.kTurningMotorGearRatio));
		inputs.turnVelocityRadsPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						turnRelativeEncoder.getVelocity()
								/ DriveConstants.TrainConstants.kTurningMotorGearRatio);
		inputs.turnAppliedVolts = turnSpark.getAppliedOutput()
				* turnSpark.getBusVoltage();
		inputs.turnSupplyCurrentAmps = turnSpark.getOutputCurrent();
		inputs.turnMotorTemp = turnSpark.getMotorTemperature();
		inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
				.mapToDouble(motorPositionRevs -> Units
						.rotationsToRadians(motorPositionRevs
								/ DriveConstants.TrainConstants.kDriveMotorGearRatio)
						* (DriveConstants.TrainConstants.kWheelDiameter / 2))
				.toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream()
				.map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void runDriveVolts(double volts) { driveSpark.setVoltage(volts); }

	@Override
	public void runTurnVolts(double volts) { turnSpark.setVoltage(volts); }

	@Override
	public void runCharacterization(double input) { runDriveVolts(input); }

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		double feedback = driveController.calculate(
				Units.rotationsPerMinuteToRadiansPerSecond(
						driveEncoder.getVelocity())
						/ DriveConstants.TrainConstants.kDriveMotorGearRatio,
				velocityRadsPerSec);
		runDriveVolts(feedback + feedForward);
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		runTurnVolts(
				turnController.calculate(absoluteEncoderValue.get().getRadians(),
						angleRads) + ff.calculate(angleRads));
	}

	@Override
	public void setDrivePID(double kP, double kI, double kD) {
		driveController.setPID(kP, kI, kD);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD, double kS) {
		turnController.setPID(kP, kI, kD);
		ff = new SimpleMotorFeedforward(kS, 0);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveSpark.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnSpark.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
	}

	@Override
	public void stop() {
		driveSpark.stopMotor();
		turnSpark.stopMotor();
	}

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			driveSpark.setSmartCurrentLimit(amps);
		});
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase(driveName, driveSpark));
		hardware.add(new SelfCheckingSparkBase(turnName, turnSpark));
		return hardware;
	}
}
