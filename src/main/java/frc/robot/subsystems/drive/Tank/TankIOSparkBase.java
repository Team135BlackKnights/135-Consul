// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;

public class TankIOSparkBase implements TankIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final CANSparkBase leftLeader;
	private final CANSparkBase rightLeader;
	private final CANSparkBase leftFollower;
	private final CANSparkBase rightFollower;
	private final RelativeEncoder leftEncoder;
	private final RelativeEncoder rightEncoder;
	private final SparkPIDController leftPID;
	private final SparkPIDController rightPID;
	private final GyroIO gyro;
	private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private static final Executor currentExecutor = Executors
			.newFixedThreadPool(8);

	public TankIOSparkBase(GyroIO gyro) {
		this.gyro = gyro;
		if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
			leftLeader = new CANSparkMax(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new CANSparkMax(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new CANSparkMax(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		} else {
			leftLeader = new CANSparkFlex(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new CANSparkFlex(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new CANSparkFlex(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new CANSparkFlex(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		}
		leftEncoder = leftLeader.getEncoder();
		rightEncoder = rightLeader.getEncoder();
		leftPID = leftLeader.getPIDController();
		rightPID = rightLeader.getPIDController();
		leftLeader.restoreFactoryDefaults();
		rightLeader.restoreFactoryDefaults();
		leftFollower.restoreFactoryDefaults();
		rightFollower.restoreFactoryDefaults();
		leftLeader.setCANTimeout(250);
		rightLeader.setCANTimeout(250);
		leftFollower.setCANTimeout(250);
		rightFollower.setCANTimeout(250);
		leftLeader.setInverted(DriveConstants.kFrontLeftDriveReversed);
		rightLeader.setInverted(DriveConstants.kFrontRightDriveReversed);
		leftFollower.follow(leftLeader, DriveConstants.kBackLeftDriveReversed);
		rightFollower.follow(rightLeader, DriveConstants.kBackRightDriveReversed);
		leftLeader.enableVoltageCompensation(12.0);
		rightLeader.enableVoltageCompensation(12.0);
		leftLeader.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		rightLeader.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		leftPID.setP(KP);
		leftPID.setD(KD);
		rightPID.setP(KP);
		rightPID.setD(KD);
		leftLeader.burnFlash();
		rightLeader.burnFlash();
		leftFollower.burnFlash();
		rightFollower.burnFlash();
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
			leftLeader.setSmartCurrentLimit(amps);
			leftFollower.setSmartCurrentLimit(amps);
			rightLeader.setSmartCurrentLimit(amps);
			rightFollower.setSmartCurrentLimit(amps);
		});
		Logger.recordOutput("Drive/CurrentLimit", amps);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		if (DriveConstants.enablePID) {
			leftPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							leftRadPerSec * GEAR_RATIO),
					ControlType.kVelocity, 0, leftFFVolts);
			rightPID.setReference(
					Units.radiansPerSecondToRotationsPerMinute(
							rightRadPerSec * GEAR_RATIO),
					ControlType.kVelocity, 0, rightFFVolts);
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
		return 12*radPerSec*(TrainConstants.kWheelDiameter/2)/DriveConstants.kMaxSpeedMetersPerSecond; 

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
