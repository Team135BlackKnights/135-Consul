// A lot of this code is borrowed from 6328's 2023 Repo.
package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;

import java.util.function.Supplier;
import frc.robot.utils.GeomUtil;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;

public class DriveToTranslation extends Command {
	private final DrivetrainS drive;
	private final boolean slowMode;
	private final Supplier<Translation2d> poseSupplier;
	private boolean running = true;
	private final ProfiledPIDController driveController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	private double driveErrorAbs;
	private PathConstraints pathConstraints;
	private Translation2d lastSetpointTranslation;
	// allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber driveKp = new LoggableTunedNumber(
			"DriveToPose/DriveKp");
	private static final LoggableTunedNumber driveKd = new LoggableTunedNumber(
			"DriveToPose/DriveKd");
	private static final LoggableTunedNumber driveMaxVelocitySlow = new LoggableTunedNumber(
			"DriveToPose/DriveMaxVelocitySlow");
	private static final LoggableTunedNumber driveTolerance = new LoggableTunedNumber(
			"DriveToPose/DriveTolerance");
	private static final LoggableTunedNumber driveToleranceSlow = new LoggableTunedNumber(
			"DriveToPose/DriveToleranceSlow");
	private static final LoggableTunedNumber ffMinRadius = new LoggableTunedNumber(
			"DriveToPose/FFMinRadius");
	private static final LoggableTunedNumber ffMaxRadius = new LoggableTunedNumber(
			"DriveToPose/FFMaxRadius");
	// Default the TunedNumbers on boot
	static {
		driveKp.initDefault(3);
		driveKd.initDefault(0.0);

		driveMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
		driveTolerance.initDefault(0.02);
		driveToleranceSlow.initDefault(0.03);
		ffMinRadius.initDefault(0.2);
		ffMaxRadius.initDefault(1);
	}

	/**
	 * Drives to the specified position under full software control. Will NOT
	 * rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, Translation2d position,
			PathConstraints pathConstraints) {
		this(drive, false, position, pathConstraints);
	}

	/**
	 * Drives to the specified pose under full software control. Will NOT rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, Translation2d pose) {
		this(drive, false, pose);
	}

	/**
	 * Drives to the specified pose under full software control. Will NOT rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, boolean slowMode, Translation2d pose) {
		this(drive, slowMode, () -> pose,
				new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.maxTranslationalAcceleration.get(),
						DriveConstants.kMaxTurningSpeedRadPerSec,
						DriveConstants.maxRotationalAcceleration.get()));
	}

	/**
	 * Drives to the specified pose under full software control. Will NOT rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, boolean slowMode, Translation2d pose,
			PathConstraints pathConstraints) {
		this(drive, slowMode, () -> pose, pathConstraints);
	}

	/**
	 * Drives to the specified pose under full software control. Will NOT rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, Supplier<Translation2d> poseSupplier) {
		this(drive, false, poseSupplier,
				new PathConstraints(DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.maxTranslationalAcceleration.get(),
						DriveConstants.kMaxTurningSpeedRadPerSec,
						DriveConstants.maxRotationalAcceleration.get()));
	}

	/**
	 * Drives to the specified pose under full software control. Will NOT rotate.
	 */
	public DriveToTranslation(DrivetrainS drive, boolean slowMode,
			Supplier<Translation2d> poseSupplier, PathConstraints pathConstraints) {
		this.drive = drive;
		this.slowMode = slowMode;
		this.poseSupplier = poseSupplier;
		this.pathConstraints = pathConstraints;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		// Reset all controllers
		running = true;
		var currentPose = drive.getPose();
		driveController.reset(
				currentPose.getTranslation()
						.getDistance(poseSupplier.get()),
				Math.min( // get our CURRENT speed, and rotate it by our actual position.
						0.0,
						-new Translation2d(drive.getFieldVelocity().dx,
								drive.getFieldVelocity().dy)
								.rotateBy(poseSupplier.get()
										.minus(drive.getPose().getTranslation())
										.getAngle().unaryMinus())
								.getX()));
		lastSetpointTranslation = drive.getPose().getTranslation();
		drive.changeDeadband(.02); // Make sure the commands aren't trying to move tiny movements when the
									// drivetrain wont allow it
		RobotContainer.currentPath = "DRIVETOPOSE";
	}

	public void updateConstraints(PathConstraints pathConstraints) {
		this.pathConstraints = pathConstraints;
		driveController.setConstraints(new TrapezoidProfile.Constraints(
				slowMode ? driveMaxVelocitySlow.get()
						: pathConstraints.maxVelocityMPS(),
				pathConstraints.maxAccelerationMPSSq()));
	}

	@Override
	public void execute() {
		// Update from tunable numbers
		if (driveMaxVelocitySlow.hasChanged(hashCode())
				|| driveTolerance.hasChanged(hashCode())
				|| driveToleranceSlow.hasChanged(hashCode())
				|| driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
			driveController.setP(driveKp.get());
			driveController.setD(driveKd.get());
			driveController.setConstraints(new TrapezoidProfile.Constraints(
					slowMode ? driveMaxVelocitySlow.get()
							: pathConstraints.maxVelocityMPS(),
					pathConstraints.maxAccelerationMPSSq()));
			driveController.setTolerance(
					slowMode ? driveToleranceSlow.get() : driveTolerance.get());
		}
		RobotContainer.currentPath = "DRIVETOPOSE";
		// Get current and target pose
		var currentPose = drive.getPose();
		var targetPose = poseSupplier.get();
		// Calculate drive speed
		double currentDistance = currentPose.getTranslation()
				.getDistance(poseSupplier.get());
		// how fast should we be moving relative to distance? use circles based off
		// relative distances to figure that out.
		double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius.get())
				/ (ffMaxRadius.get() - ffMinRadius.get()), 0.0, 1.0);
		driveErrorAbs = currentDistance;
		driveController.reset(
				lastSetpointTranslation.getDistance(targetPose),
				driveController.getSetpoint().velocity);
		double driveVelocityScalar = driveController.getSetpoint().velocity
				* ffScaler + driveController.calculate(driveErrorAbs, 0.0); // Go to error of zero from wanted pose,
																			// using ff
		if (currentDistance < driveController.getPositionTolerance())
			driveVelocityScalar = 0.0; // if there, STOP.
		lastSetpointTranslation = new Pose2d(targetPose,
				currentPose.getTranslation().minus(targetPose)
						.getAngle())
				.transformBy(GeomUtil.translationToTransform(
						driveController.getSetpoint().position, 0.0))
				.getTranslation();
		var driveVelocity = new Pose2d(new Translation2d(),
				currentPose.getTranslation().minus(targetPose)
						.getAngle())
				.transformBy(GeomUtil
						.translationToTransform(driveVelocityScalar, 0.0))
				.getTranslation(); // Calculate X and Y speeds from driveVelocity scalar.
		drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
				driveVelocity.getX(), driveVelocity.getY(), RobotContainer.angularSpeed,
				currentPose.getRotation())); // assert that we are relative to the current pose
		// Log data
		Logger.recordOutput("DriveToPose/DistanceError", currentDistance);
		Logger.recordOutput("DriveToPose/DistanceSetpoint",
				driveController.getSetpoint().position);
		Logger.recordOutput("Odometry/DriveToPoseSetpoint",
				new Pose2d(lastSetpointTranslation,
						currentPose.getRotation()));
		Logger.recordOutput("Odometry/DriveToPoseGoal", targetPose);
		if (atGoal())
			running = false; // If we've reached our goal, stop command.
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angularSpeed = 0;

		drive.changeDeadband(DriveConstants.TrainConstants.kDeadband); // go back to normal deadband
		drive.stopModules();
	}

	/** Checks if the robot is stopped at the final pose. */
	public boolean atGoal() {
		return driveController.atGoal();
	}

	/**
	 * Checks if the robot pose is within the custom drive and theta tolerances.
	 */
	public boolean withinTolerance(double driveTolerance,
			Rotation2d thetaTolerance) {
		return Math.abs(driveErrorAbs) < driveTolerance;

	}

	@Override
	public boolean isFinished() {
		return !running;
	}
}