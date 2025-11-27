// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;

public class TankIOSparkBase implements TankIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatioLow;
	private static final double KP = DriveConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.overallDriveMotorConstantContainer
			.getD();
	private final SparkBase leftLeader;
	private final SparkBase rightLeader;
	private final SparkBase leftFollower;
	private final SparkBase rightFollower;
	private final RelativeEncoder leftEncoder;
	private final RelativeEncoder rightEncoder;
	private final SparkClosedLoopController leftPID;
	private final SparkClosedLoopController rightPID;
	private final GyroIO gyro;
	private final SparkBaseConfig sparkConfig;
	private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(8);

	public TankIOSparkBase(GyroIO gyro) {
		this.gyro = gyro;
		if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
			leftLeader = new SparkMax(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new SparkMax(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new SparkMax(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new SparkMax(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
			sparkConfig = new SparkMaxConfig();
		} else {
			leftLeader = new SparkFlex(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new SparkFlex(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new SparkFlex(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new SparkFlex(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
			sparkConfig = new SparkFlexConfig();
		}
		leftEncoder = leftLeader.getEncoder();
		rightEncoder = rightLeader.getEncoder();
		leftPID = leftLeader.getClosedLoopController();
		rightPID = rightLeader.getClosedLoopController();
		leftLeader.setCANTimeout(250);
		rightLeader.setCANTimeout(250);
		leftFollower.setCANTimeout(250);
		rightFollower.setCANTimeout(250);
		sparkConfig.inverted(DriveConstants.kFrontLeftDriveReversed);
		sparkConfig.voltageCompensation(12);
		sparkConfig.smartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		ClosedLoopConfig loopConfig = new ClosedLoopConfig();
		loopConfig.p(KP);
		loopConfig.d(KD);
		sparkConfig.apply(loopConfig);
		leftLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.inverted(DriveConstants.kFrontRightDriveReversed);
		rightLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.follow(leftLeader);
		leftFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		sparkConfig.follow(rightLeader);
		rightFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		gyro.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		inputs.leftPositionRad = Units
				.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
				leftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftAppliedVolts = leftLeader.getAppliedOutput()
				* leftLeader.getBusVoltage();
		inputs.leftCurrentAmps = new double[] { leftLeader.getOutputCurrent(),
				leftFollower.getOutputCurrent()
		};
		inputs.frontLeftDriveTemp = leftLeader.getMotorTemperature();
		inputs.backLeftDriveTemp = leftFollower.getMotorTemperature();
		inputs.rightPositionRad = Units
				.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						rightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightAppliedVolts = rightLeader.getAppliedOutput()
				* rightLeader.getBusVoltage();
		inputs.rightCurrentAmps = new double[] { rightLeader.getOutputCurrent(),
				rightFollower.getOutputCurrent()
		};
		inputs.frontRightDriveTemp = rightLeader.getMotorTemperature();
		inputs.backRightDriveTemp = rightFollower.getMotorTemperature();
		inputs.gyroConnected = gyroInputs.connected;
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void reset() { gyro.reset(); }

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
	}

	@Override
	public void setCurrentLimit(int amps) {
		currentExecutor.execute(() -> {
			sparkConfig.smartCurrentLimit(amps);
			leftLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			rightLeader.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			sparkConfig.follow(leftLeader);
			leftFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
			sparkConfig.follow(rightLeader);
			rightFollower.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		});
		Logger.recordOutput("Drive/CurrentLimit", amps);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		if (DriveConstants.enablePID) {
			leftPID.setSetpoint(
					Units.radiansPerSecondToRotationsPerMinute(
							leftRadPerSec * GEAR_RATIO),
					ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
			rightPID.setSetpoint(
					Units.radiansPerSecondToRotationsPerMinute(
							rightRadPerSec * GEAR_RATIO),
					ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
		} else {
			setVoltage(convertRadPerSecondToVoltage(leftRadPerSec),
					convertRadPerSecondToVoltage(rightRadPerSec));
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
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.addAll(gyro.getSelfCheckingHardware());
		hardware.add(new SelfCheckingSparkBase("FrontLeftDrive", leftLeader));
		hardware.add(new SelfCheckingSparkBase("BackLeftDrive", leftFollower));
		hardware.add(new SelfCheckingSparkBase("FrontRightDrive", rightLeader));
		hardware.add(new SelfCheckingSparkBase("BackRightDrive", rightFollower));
		return hardware;
	}
}
