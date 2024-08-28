// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;

public class TankIOSim implements TankIO {
	public final TankDrivePhysicsSimResults tankDrivePhysicsSimResults = new TankDrivePhysicsSimResults();
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter
			/ 2;
	private DCMotorSim frontLeft = new DCMotorSim(
			DriveConstants.getDriveTrainMotors(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim frontRight = new DCMotorSim(
			DriveConstants.getDriveTrainMotors(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim backLeft = new DCMotorSim(
			DriveConstants.getDriveTrainMotors(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim backRight = new DCMotorSim(
			DriveConstants.getDriveTrainMotors(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private double leftAppliedVolts = 0.0;
	private double rightAppliedVolts = 0.0;
	private boolean closedLoop = false;
	private PIDController leftPID = new PIDController(KP, 0.0, KD);
	private PIDController rightPID = new PIDController(KP, 0.0, KD);
	private double leftFFVolts = 0.0;
	private double rightFFVolts = 0.0;
	private final GyroIO gyro;
	private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	@Override
	public void updateSim(double dtSeconds) {
		frontLeft.update(dtSeconds);
		backLeft.update(dtSeconds);
		frontRight.update(dtSeconds);
		backRight.update(dtSeconds);
	}

	public TankIOSim(GyroIO gyroSim) { gyro = gyroSim; }

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				((frontLeft.getAngularVelocityRadPerSec() * WHEEL_RADIUS
						+ backLeft.getAngularVelocityRadPerSec() * WHEEL_RADIUS) / 2),
				((frontRight.getAngularVelocityRadPerSec() * WHEEL_RADIUS
						+ backRight.getAngularVelocityRadPerSec() * WHEEL_RADIUS)
						/ 2));
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		gyro.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		if (closedLoop) {
			leftAppliedVolts = MathUtil
					.clamp(leftPID.calculate((frontLeft.getAngularVelocityRadPerSec()
							+ backLeft.getAngularVelocityRadPerSec()) / 2
							) + leftFFVolts, -12.0, 12.0);
			rightAppliedVolts = MathUtil
					.clamp(
							rightPID
									.calculate((frontRight.getAngularVelocityRadPerSec()
											+ backRight.getAngularVelocityRadPerSec()) / 2
											)
									+ rightFFVolts,
							-12.0, 12.0);
			// Set inputs to the motors
			frontLeft.setInputVoltage(leftAppliedVolts);
			backLeft.setInputVoltage(leftAppliedVolts);
			frontRight.setInputVoltage(rightAppliedVolts);
			backRight.setInputVoltage(rightAppliedVolts);
		}
		// Populate inputs
		inputs.leftPositionRad =tankDrivePhysicsSimResults.driveWheelFinalRevolutions[0]
		* 2 * Math.PI * 4;
		inputs.leftVelocityRadPerSec = tankDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[0]* 2 * Math.PI * 4;
		inputs.leftAppliedVolts = leftAppliedVolts;
		inputs.leftCurrentAmps = new double[] { frontLeft.getCurrentDrawAmps(),
				backLeft.getCurrentDrawAmps()
		};
		inputs.rightPositionRad = tankDrivePhysicsSimResults.driveWheelFinalRevolutions[1]* 2 * Math.PI * 4;
		inputs.rightVelocityRadPerSec = tankDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[1]* 2 * Math.PI * 4;
		inputs.rightAppliedVolts = rightAppliedVolts;
		inputs.rightCurrentAmps = new double[] { frontRight.getCurrentDrawAmps(),
				backRight.getCurrentDrawAmps()
		};
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.gyroConnected = gyroInputs.connected;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		closedLoop = false;
		leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
		rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		closedLoop = true;
		leftPID.setSetpoint(leftRadPerSec);
		rightPID.setSetpoint(rightRadPerSec);
		this.leftFFVolts = leftFFVolts;
		this.rightFFVolts = rightFFVolts;
	}

	public static class TankDrivePhysicsSimResults {
		public double[] driveWheelFinalRevolutions = { 0, 0,
		}, driveWheelFinalVelocityRevolutionsPerSec = { 0, 0,
		};
		public boolean[] negateFF = { false, false
		};
		public final double[][] odometryDriveWheelRevolutions = new double[][] {
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD],
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD],
		};

		public TankDrivePhysicsSimResults() {
			for (double[] odometryDriveWheelRevolution : odometryDriveWheelRevolutions)
				Arrays.fill(odometryDriveWheelRevolution, 0);
		}
	}
}
