// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;

public class MecanumIOSparkBase implements MecanumIO {

	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatioLow;
	private static final double KP = DriveConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.overallDriveMotorConstantContainer
			.getD();
	private final SparkBaseConfig sparkConfig;
	private final SparkBase frontLeft;
	private final SparkBase frontRight;
	private final SparkBase backLeft;
	private final SparkBase backRight;
	private final RelativeEncoder frontLeftEncoder;
	private final RelativeEncoder frontRightEncoder;
	private final RelativeEncoder backLeftEncoder;
	private final RelativeEncoder backRightEncoder;
	private final SparkClosedLoopController frontLeftPID;
	private final SparkClosedLoopController frontRightPID;
	private final SparkClosedLoopController backLeftPID;
	private final SparkClosedLoopController backRightPID;
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(8);

	public MecanumIOSparkBase(GyroIO gyroIO) {
		this.gyroIO = gyroIO;
		if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
			frontLeft = new SparkMax(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			frontRight = new SparkMax(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			backLeft = new SparkMax(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			backRight = new SparkMax(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
			sparkConfig = new SparkMaxConfig();
		} else {
			frontLeft = new SparkFlex(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			frontRight = new SparkFlex(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			backLeft = new SparkFlex(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			backRight = new SparkFlex(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
			sparkConfig = new SparkFlexConfig();
		}

		frontLeft.setCANTimeout(250);
		frontRight.setCANTimeout(250);
		backLeft.setCANTimeout(250);
		backRight.setCANTimeout(250);
		sparkConfig.inverted(DriveConstants.kFrontLeftDriveReversed);
		sparkConfig.voltageCompensation(12);
		sparkConfig.smartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		sparkConfig.idleMode(IdleMode.kBrake);
		frontLeftEncoder = frontLeft.getEncoder();
		frontRightEncoder = frontRight.getEncoder();
		backLeftEncoder = backLeft.getEncoder();
		backRightEncoder = backRight.getEncoder();
		frontLeftPID = frontLeft.getClosedLoopController();
		frontRightPID = frontRight.getClosedLoopController();
		backLeftPID = backLeft.getClosedLoopController();
		backRightPID = backRight.getClosedLoopController();
		ClosedLoopConfig loopConfig = new ClosedLoopConfig();
		loopConfig.d(KD);
		loopConfig.p(KP);
		MAXMotionConfig MaxMotionConfig = new MAXMotionConfig();
		MaxMotionConfig.maxVelocity(Units.radiansPerSecondToRotationsPerMinute(TrainConstants.kMaxAngularSpeedRadiansPerSecond));
		sparkConfig.apply(loopConfig);
		frontLeft.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.inverted(DriveConstants.kFrontRightDriveReversed);
		frontRight.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.inverted(DriveConstants.kBackLeftDriveReversed);
		backLeft.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.inverted(DriveConstants.kBackRightDriveReversed);
		backRight.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		

	}

	@Override
	public void updateInputs(MecanumIOInputs inputs) {
		inputs.leftFrontPositionRad = Units
				.rotationsToRadians(frontLeftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftFrontVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						frontLeftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftFrontAppliedVolts = frontLeft.getAppliedOutput()
				* frontLeft.getBusVoltage();
		inputs.leftBackPositionRad = Units
				.rotationsToRadians(backLeftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftBackVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						backLeftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftBackAppliedVolts = backLeft.getAppliedOutput()
				* backLeft.getBusVoltage();
		inputs.leftCurrentAmps = new double[] { frontLeft.getOutputCurrent(),
				backLeft.getOutputCurrent()
		};
		inputs.frontLeftDriveTemp = frontLeft.getMotorTemperature();
		inputs.backLeftDriveTemp = backLeft.getMotorTemperature();
		inputs.rightFrontPositionRad = Units
				.rotationsToRadians(frontRightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightFrontVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						frontRightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightFrontAppliedVolts = frontRight.getAppliedOutput()
				* frontRight.getBusVoltage();
		inputs.rightBackPositionRad = Units
				.rotationsToRadians(backRightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightBackVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						backRightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightBackAppliedVolts = backRight.getAppliedOutput()
				* backRight.getBusVoltage();
		inputs.rightCurrentAmps = new double[] { frontRight.getOutputCurrent(),
				backRight.getOutputCurrent()
		};
		inputs.frontRightDriveTemp = frontRight.getMotorTemperature();
		inputs.backRightDriveTemp = backRight.getMotorTemperature();
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		inputs.gyroConnected = gyroInputs.connected;
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void reset() { gyroIO.reset(); }

	@Override
	public void setVoltage(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		frontLeft.setVoltage(frontLeftVolts);
		frontRight.setVoltage(frontRightVolts);
		backLeft.setVoltage(backLeftVolts);
		backRight.setVoltage(backRightVolts);
	}

	@Override
	public void setVelocity(double frontLeftRadPerSec,
			double frontRightRadPerSec, double backLeftRadPerSec,
			double backRightRadPerSec, double frontLeftFFVolts,
			double frontRightFFVolts, double backLeftFFVolts,
			double backRightFFVolts) {
		if (DriveConstants.enablePID) {
			frontLeftPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							frontLeftRadPerSec * GEAR_RATIO),
					ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, frontLeftFFVolts);
			frontRightPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							frontRightRadPerSec * GEAR_RATIO),
					ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, frontRightFFVolts);
			backLeftPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							backLeftRadPerSec * GEAR_RATIO),
					ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, backLeftFFVolts);
			backRightPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							backRightRadPerSec * GEAR_RATIO),
					ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, backRightFFVolts);
		}else{
			setVoltage(convertRadPerSecondToVoltage(frontLeftRadPerSec), convertRadPerSecondToVoltage(frontRightRadPerSec), convertRadPerSecondToVoltage(backLeftRadPerSec), convertRadPerSecondToVoltage(backRightRadPerSec));
		}
	}

	/** Converts radians per second into voltage that will achieve that value in a motor.
	 * Takes the angular velocity of the motor (radPerSec),
	 * divides by the theoretical max angular speed (max linear speed / wheel radius)
	 * and multiplies by 12 (the theoretical standard voltage)  
	 * @param radPerSec radians per second of the motor
	 * @return the voltage that should be sent to the motor
	 */
	public double convertRadPerSecondToVoltage(double radPerSec) {
		return 12*radPerSec*(TrainConstants.kWheelDiameter.get()/2)/DriveConstants.kMaxSpeedMetersPerSecond; 

	}

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			sparkConfig.smartCurrentLimit(amps);
			frontLeft.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			frontRight.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			backLeft.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			backRight.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		});
		Logger.recordOutput("Mecanum/CurrentLimit", amps);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.addAll(gyroIO.getSelfCheckingHardware());
		hardware.add(new SelfCheckingSparkBase("FrontLeftDrive", frontLeft));
		hardware.add(new SelfCheckingSparkBase("BackLeftDrive", backLeft));
		hardware.add(new SelfCheckingSparkBase("FrontRightDrive", frontRight));
		hardware.add(new SelfCheckingSparkBase("BackRightDrive", backRight));
		return hardware;
	}
}
