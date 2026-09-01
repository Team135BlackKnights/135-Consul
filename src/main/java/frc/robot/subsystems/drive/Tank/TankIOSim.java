// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.kinematics.DifferentialDriveWheelVelocities;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;

public class TankIOSim implements TankIO {
	public final TankDrivePhysicsSimResults tankDrivePhysicsSimResults = new TankDrivePhysicsSimResults();
	private static final double KP = DriveConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.overallDriveMotorConstantContainer
			.getD();
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter.get()
			/ 2;
	private final DCMotorSim frontLeft, backLeft, frontRight, backRight;
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

	public TankIOSim(GyroIO gyroSim) {
		gyro = gyroSim;
		frontLeft = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(DriveConstants.getDriveTrainMotors(1), .01, DriveConstants.TrainConstants.kDriveMotorGearRatioLow), DriveConstants.getDriveTrainMotors(1), .1,.1);
		backLeft = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(DriveConstants.getDriveTrainMotors(1), .01, DriveConstants.TrainConstants.kDriveMotorGearRatioLow), DriveConstants.getDriveTrainMotors(1), .1,.1);
		frontRight = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(DriveConstants.getDriveTrainMotors(1), .01, DriveConstants.TrainConstants.kDriveMotorGearRatioLow), DriveConstants.getDriveTrainMotors(1), .1,.1);
		backRight = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(DriveConstants.getDriveTrainMotors(1), .01, DriveConstants.TrainConstants.kDriveMotorGearRatioLow), DriveConstants.getDriveTrainMotors(1), .1,.1);
		
	}

	public DifferentialDriveWheelVelocities getWheelSpeeds() {
		return new DifferentialDriveWheelVelocities(
				((frontLeft.getAngularVelocity() * WHEEL_RADIUS
						+ backLeft.getAngularVelocity() * WHEEL_RADIUS) / 2),
				((frontRight.getAngularVelocity() * WHEEL_RADIUS
						+ backRight.getAngularVelocity() * WHEEL_RADIUS)
						/ 2));
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		gyro.updateInputs(gyroInputs);
		Logger.processInputs("Gyro", gyroInputs);
		if (closedLoop) {
			leftAppliedVolts = frc.robot.utils.maths.CommonMath
					.clamp(leftPID.calculate((frontLeft.getAngularVelocity()
							+ backLeft.getAngularVelocity()) / 2) + leftFFVolts, -12.0, 12.0);
			rightAppliedVolts = frc.robot.utils.maths.CommonMath
					.clamp(
							rightPID
									.calculate((frontRight.getAngularVelocity()
											+ backRight.getAngularVelocity()) / 2)
									+ rightFFVolts,
							-12.0, 12.0);
			// Set inputs to the motors
			frontLeft.setInputVoltage(leftAppliedVolts);
			backLeft.setInputVoltage(leftAppliedVolts);
			frontRight.setInputVoltage(rightAppliedVolts);
			backRight.setInputVoltage(rightAppliedVolts);
		}
		// Populate inputs
		inputs.leftPositionRad = tankDrivePhysicsSimResults.driveWheelFinalRevolutions[0]
				* 2 * Math.PI * 4;
		inputs.leftVelocityRadPerSec = tankDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[0] * 2
				* Math.PI * 4;
		inputs.leftAppliedVolts = leftAppliedVolts;
		inputs.leftCurrentAmps = new double[] { frontLeft.getCurrentDraw(),
				backLeft.getCurrentDraw()
		};
		inputs.rightPositionRad = tankDrivePhysicsSimResults.driveWheelFinalRevolutions[1] * 2 * Math.PI * 4;
		inputs.rightVelocityRadPerSec = tankDrivePhysicsSimResults.driveWheelFinalVelocityRevolutionsPerSec[1] * 2
				* Math.PI * 4;
		inputs.rightAppliedVolts = rightAppliedVolts;
		inputs.rightCurrentAmps = new double[] { frontRight.getCurrentDraw(),
				backRight.getCurrentDraw()
		};
		inputs.gyroYaw = gyroInputs.yawPosition;
		inputs.gyroConnected = gyroInputs.connected;
		inputs.collisionDetected = gyroInputs.collisionDetected;
	}

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		closedLoop = false;
		leftAppliedVolts = frc.robot.utils.maths.CommonMath.clamp(leftVolts, -12.0, 12.0);
		rightAppliedVolts = frc.robot.utils.maths.CommonMath.clamp(rightVolts, -12.0, 12.0);
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
