package frc.robot.subsystems.drive.FastSwerve;

import java.util.Arrays;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;
import static frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs.*;

public class ModuleIOSim implements ModuleIO {
	public final SwerveModulePhysicsSimulationResults physicsSimulationResults = new SwerveModulePhysicsSimulationResults();
	private final DCMotorSim driveSim = new DCMotorSim(DRIVE_MOTOR,
			DriveConstants.TrainConstants.kDriveMotorGearRatio,
			DRIVE_WHEEL_ROTTER_INERTIA);
	private final DCMotorSim turnSim = new DCMotorSim(STEER_MOTOR,
			DriveConstants.TrainConstants.kTurningMotorGearRatio, STEER_INERTIA);
	private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);
	private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;
	private final Rotation2d turnAbsoluteInitPosition;
	private boolean driveCoast = false;
	private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);
	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer
					.getKs(),
			0);

	public ModuleIOSim(int index) {
		switch (index) {
		case 0:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		case 1:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		case 2:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		default:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		}
		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		if (DriverStation.isDisabled()) {
			stop();
		}
		if (driveCoast && DriverStation.isDisabled()) {
			runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
		} else {
			driveVoltsLimiter.reset(driveAppliedVolts);
		}
		inputs.negateFF = physicsSimulationResults.negateFF;
		inputs.drivePositionRads = physicsSimulationResults.driveWheelFinalRevolutions
				* 2 * Math.PI * 4;
		inputs.driveVelocityRadsPerSec = physicsSimulationResults.driveWheelFinalVelocityRevolutionsPerSec
				* 2 * Math.PI * 4;
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
		inputs.turnAbsolutePosition = new Rotation2d(
				turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
		inputs.turnPosition = Rotation2d
				.fromRadians(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
		double[] odometryDrivePositionsMeters = new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
		for (int i = 0; i < SIM_ITERATIONS_PER_ROBOT_PERIOD; i++) {
			odometryDrivePositionsMeters[i] = Units.rotationsToRadians(
					physicsSimulationResults.odometryDriveWheelRevolutions[i])
					* DriveConstants.TrainConstants.kWheelDiameter / 2 * 4;
		}
		inputs.odometryDrivePositionsMeters = Arrays.copyOf(
				odometryDrivePositionsMeters, SIM_ITERATIONS_PER_ROBOT_PERIOD);
		inputs.odometryTurnPositions = new Rotation2d[] {
				Rotation2d.fromRadians(turnSim.getAngularPositionRad())
		};
	}

	public void runDriveVolts(double volts) {
		driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		driveSim.setInputVoltage(driveAppliedVolts);
	}

	public void runTurnVolts(double volts) {
		turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		turnSim.setInputVoltage(turnAppliedVolts);
	}

	@Override
	public void runCharacterization(double input) { runDriveVolts(input); }

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		runDriveVolts(
				driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(),
						velocityRadsPerSec) + feedForward);
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		runTurnVolts(
				turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads)
						+ ff.calculate(angleRads));
	}

	@Override
	public void setDrivePID(double kP, double kI, double kD) {
		driveFeedback.setPID(kP, kI, kD);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD, double kS) {
		turnFeedback.setPID(kP, kI, kD);
		ff = new SimpleMotorFeedforward(kS, 0);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) { driveCoast = !enable; }

	public void updateSim(double periodSecs) {
		turnSim.update(periodSecs);
		driveSim.update(periodSecs);
	}

	/**
	 * gets the swerve state, assuming that the chassis is allowed to move freely
	 * on field (not hitting anything)
	 * 
	 * @return the swerve state, in percent full speed
	 */
	public SwerveModuleState getFreeSwerveSpeed(double robotMaximumFloorSpeed) {
		return new SwerveModuleState(
				driveSim.getAngularVelocityRPM() / DRIVE_MOTOR_FREE_FINAL_SPEED_RPM
						* robotMaximumFloorSpeed,
				Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
	}

	/**
	 * this replaces DC Motor Sim for drive wheels
	 */
	public static class SwerveModulePhysicsSimulationResults {
		public double driveWheelFinalRevolutions = 0,
				driveWheelFinalVelocityRevolutionsPerSec = 0;
		public boolean negateFF = false;
		public final double[] odometryDriveWheelRevolutions = new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
		public final Rotation2d[] odometrySteerPositions = new Rotation2d[SIM_ITERATIONS_PER_ROBOT_PERIOD];

		public SwerveModulePhysicsSimulationResults() {
			Arrays.fill(odometrySteerPositions, new Rotation2d());
			Arrays.fill(odometryDriveWheelRevolutions, 0);
		}
	}

	@Override
	public void stop() {
		runDriveVolts(0.0);
		runTurnVolts(0.0);
	}
}
