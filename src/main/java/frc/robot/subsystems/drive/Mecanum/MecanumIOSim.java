// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;

public class MecanumIOSim implements MecanumIO {
	public final MecanumDrivePhysicsSimResults mecanumDrivePhysicsSimResults = new MecanumDrivePhysicsSimResults();
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
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
	private double frontLeftAppliedVolts = 0.0;
	private double frontRightAppliedVolts = 0.0;
	private double backLeftAppliedVolts = 0.0;
	private double backRightAppliedVolts = 0.0;
	private boolean closedLoop = false;
	private PIDController frontLeftPID = new PIDController(KP, 0.0, KD);
	private PIDController frontRightPID = new PIDController(KP, 0.0, KD);
	private PIDController backLeftPID = new PIDController(KP, 0.0, KD);
	private PIDController backRightPID = new PIDController(KP, 0.0, KD);
	private double frontLeftFFVolts = 0.0;
	private double frontRightFFVolts = 0.0;
	private double backLeftFFVolts = 0.0;
	private double backRightFFVolts = 0.0;
	private final GyroIO gyro;
	private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter
			/ 2;

	public MecanumIOSim(GyroIO gyroSim) { gyro = gyroSim; }

	@Override
	public void updateSim(double dtSeconds) {
		frontLeft.update(dtSeconds);
		frontRight.update(dtSeconds);
		backLeft.update(dtSeconds);
		backRight.update(dtSeconds);
	}

	@Override
	public void updateInputs(MecanumIOInputs inputs) {
		gyro.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		if (closedLoop) {
			frontLeftAppliedVolts = MathUtil.clamp(
					frontLeftPID.calculate(frontLeft.getAngularVelocityRadPerSec())
							+ frontLeftFFVolts,
					-12.0, 12.0);
			frontRightAppliedVolts = MathUtil.clamp(
					frontRightPID.calculate(frontRight.getAngularVelocityRadPerSec())
							+ frontRightFFVolts,
					-12.0, 12.0);
			backLeftAppliedVolts = MathUtil.clamp(
					backLeftPID.calculate(backLeft.getAngularVelocityRadPerSec())
							+ backLeftFFVolts,
					-12.0, 12.0);
			backRightAppliedVolts = MathUtil.clamp(
					backRightPID.calculate(backRight.getAngularVelocityRadPerSec())
							+ backRightFFVolts,
					-12.0, 12.0);
			frontLeft.setInputVoltage(frontLeftAppliedVolts);
			frontRight.setInputVoltage(frontRightAppliedVolts);
			backLeft.setInputVoltage(backLeftAppliedVolts);
			backRight.setInputVoltage(backRightAppliedVolts);
		}
		// Update gyro simulation (you might want to base this on your robot's movement)
		//Pigeon2SimState simState = pigeon.getSimState();
		//double angularVelocity = (frontLeft.getAngularVelocityRadPerSec() - frontRight.getAngularVelocityRadPerSec()
		//	 + backLeft.getAngularVelocityRadPerSec() - backRight.getAngularVelocityRadPerSec()) / 4.0;
		//simState.addYaw(Units.radiansToDegrees(angularVelocity));
		inputs.leftFrontPositionRad = mecanumDrivePhysicsSimResults.driveWheelFinalRevolutions[0]
				* 2 * Math.PI * 4;
		inputs.leftFrontVelocityRadPerSec = mecanumDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[0]
				* 2 * Math.PI * 4;
		inputs.leftFrontAppliedVolts = frontLeftAppliedVolts;
		inputs.leftBackPositionRad = mecanumDrivePhysicsSimResults.driveWheelFinalRevolutions[2]
				* 2 * Math.PI * 4;
		inputs.leftBackVelocityRadPerSec = mecanumDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[2]
				* 2 * Math.PI * 4;
		inputs.leftBackAppliedVolts = backLeftAppliedVolts;
		inputs.leftCurrentAmps = new double[] { frontLeft.getCurrentDrawAmps(),
				backLeft.getCurrentDrawAmps()
		};
		inputs.rightFrontPositionRad = mecanumDrivePhysicsSimResults.driveWheelFinalRevolutions[1]
				* 2 * Math.PI * 4;
		inputs.rightFrontVelocityRadPerSec = mecanumDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[1]
				* 2 * Math.PI * 4;
		inputs.rightFrontAppliedVolts = frontRightAppliedVolts;
		inputs.rightBackPositionRad = mecanumDrivePhysicsSimResults.driveWheelFinalRevolutions[3]
				* 2 * Math.PI * 4;
		inputs.rightBackVelocityRadPerSec = mecanumDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[3]
				* 2 * Math.PI * 4;
		inputs.rightBackAppliedVolts = backRightAppliedVolts;
		inputs.rightCurrentAmps = new double[] { frontRight.getCurrentDrawAmps(),
				backRight.getCurrentDrawAmps()
		};
		inputs.gyroConnected = gyroInputs.connected;
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void setVoltage(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		closedLoop = false;
		frontLeftAppliedVolts = MathUtil.clamp(frontLeftVolts, -12.0, 12.0);
		frontRightAppliedVolts = MathUtil.clamp(frontRightVolts, -12.0, 12.0);
		backLeftAppliedVolts = MathUtil.clamp(backLeftVolts, -12.0, 12.0);
		backRightAppliedVolts = MathUtil.clamp(backRightVolts, -12.0, 12.0);
		frontLeft.setInputVoltage(frontLeftVolts);
		frontRight.setInputVoltage(frontRightVolts);
		backLeft.setInputVoltage(backLeftVolts);
		backRight.setInputVoltage(backRightVolts);
	}

	@Override
	public void setVelocity(double frontLeftRadPerSec,
			double frontRightRadPerSec, double backLeftRadPerSec,
			double backRightRadPerSec, double frontLeftFFVolts,
			double frontRightFFVolts, double backLeftFFVolts,
			double backRightFFVolts) {
		closedLoop = true;
		frontLeftPID.setSetpoint(frontLeftRadPerSec);
		frontRightPID.setSetpoint(frontRightRadPerSec);
		backLeftPID.setSetpoint(backLeftRadPerSec);
		backRightPID.setSetpoint(backRightRadPerSec);
		this.frontLeftFFVolts = frontLeftFFVolts;
		this.frontRightFFVolts = frontRightFFVolts;
		this.backLeftFFVolts = backLeftFFVolts;
		this.backRightFFVolts = backRightFFVolts;
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds() {
		return new MecanumDriveWheelSpeeds(
				frontLeft.getAngularVelocityRadPerSec() * WHEEL_RADIUS,
				frontRight.getAngularVelocityRadPerSec() * WHEEL_RADIUS,
				backLeft.getAngularVelocityRadPerSec() * WHEEL_RADIUS,
				backRight.getAngularVelocityRadPerSec() * WHEEL_RADIUS);
	}

	public static class MecanumDrivePhysicsSimResults {
		public double[] driveWheelFinalRevolutions = { 0, 0, 0, 0
		}, driveWheelFinalVelocityRevolutionsPerSec = { 0, 0, 0, 0
		};
		public boolean[] negateFF = { false, false, false, false
		};
		public final double[][] odometryDriveWheelRevolutions = new double[][] {
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD],
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD],
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD],
				new double[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD]
		};

		public MecanumDrivePhysicsSimResults() {
			for (double[] odometryDriveWheelRevolution : odometryDriveWheelRevolutions)
				Arrays.fill(odometryDriveWheelRevolution, 0);
		}
	}
}
