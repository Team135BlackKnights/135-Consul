// A lot of this code is borrowed from 6328's 2023 Repo.
package frc.robot.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;

public class DriveToTranslation extends Command {
	private final DrivetrainS drive;
	private final boolean isAuto;
	private final Supplier<Translation2d> poseSupplier;
	private Supplier<Pose2d> currentPoseSupplier;
	private boolean running = true;
	private double tolerance = 0.05;
	private final ProfiledPIDController driveController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	private double driveErrorAbs;
	private PathConstraints pathConstraints;
	private Translation2d lastSetpointTranslation;
	// allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber driveKp = new LoggableTunedNumber(
			"DriveToPose/DriveKp");
	private static final LoggableTunedNumber driveKi = new LoggableTunedNumber(
			"DriveToPose/DriveKi");
	private static final LoggableTunedNumber driveKd = new LoggableTunedNumber(
			"DriveToPose/DriveKd");
	private static final LoggableTunedNumber driveMaxVelocitySlow = new LoggableTunedNumber(
			"DriveToPose/DriveMaxVelocitySlow");
	private static final LoggableTunedNumber ffMinRadius = new LoggableTunedNumber(
			"DriveToPose/FFMinRadius");
	private static final LoggableTunedNumber ffMaxRadius = new LoggableTunedNumber(
			"DriveToPose/FFMaxRadius");
	private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
	private boolean hasPose = false;
	// Default the TunedNumbers on boot
	static {
		driveKp.initDefault(3, TuningConstants.isTuningMacros); // old 1
		driveKi.initDefault(0, TuningConstants.isTuningMacros); // old .5
		driveKd.initDefault(0.0, TuningConstants.isTuningMacros); // old .125

		driveMaxVelocitySlow.initDefault(6, TuningConstants.isTuningMacros);
		ffMinRadius.initDefault(.125, TuningConstants.isTuningMacros); // old .9
		ffMaxRadius.initDefault(2, TuningConstants.isTuningMacros); // old 3
	}

	/**
	 * Drives to the specified position under full software control. Will NOT
	 * rotate.
	 *
	 * @param drive The drivetrain subsystem.
	 * @param args  Flexible arguments for initialization:
	 * 
	 *              <ul>
	 *              <li><b>Position: (pick ONE)</b>
	 *              <ul>
	 *              <li>`Translation2d`: Uses the target translation.</li>
	 *              <li>`Supplier<Translation2d>`: Dynamically provides the
	 *              translation during execution.</li>
	 *              </ul>
	 *              </li>
	 *              <li><b>PathConstraints: (optional)</b> Specifies the path
	 *              constraints to follow,
	 *              otherwise uses default values.</li>
	 *              <li><b>Boolean: (optional)</b> Enables slow mode if true,
	 *              otherwise false by default.</li>
	 *              </ul>
	 */
	@SafeVarargs
	public DriveToTranslation(DrivetrainS drive, Object... args) {
		this.drive = drive;

		// Default values
		Supplier<Translation2d> poseSupplier = () -> new Translation2d();
		PathConstraints pathConstraints = new PathConstraints(
				DriveConstants.kMaxSpeedMetersPerSecond,
				DriveConstants.maxTranslationalAcceleration.get(),
				DriveConstants.kMaxTurningSpeedRadPerSec,
				DriveConstants.maxRotationalAcceleration.get());
		boolean isAuto = false;
		double newTolerance = Units.inchesToMeters(1); // Default tolerance
		Supplier<Pose2d> currentPoseSupplier = () -> drive.getPose();
		// Parse arguments
		for (Object arg : args) {
			if (arg instanceof Translation2d translation) {
				poseSupplier = () -> translation;
			} else if (arg instanceof Supplier<?> supplier && supplier.get() instanceof Translation2d) {
				if (hasPose) {
					this.linearFF = () -> (Translation2d) supplier.get();
				} else {
					poseSupplier = () -> (Translation2d) supplier.get();
					hasPose = true;
				}

			} else if (arg instanceof Supplier<?> supplier && supplier.get() instanceof Pose2d) {
				currentPoseSupplier = () -> ((Pose2d) supplier.get());
			} else if (arg instanceof Pose2d pose) {
				currentPoseSupplier = () -> pose;
			} else if (arg instanceof Double tolerance) {
				newTolerance = tolerance;
			} else if (arg instanceof PathConstraints constraints) {
				pathConstraints = constraints;
			} else if (arg instanceof Boolean mode) {
				isAuto = mode;
			} else {
				throw new IllegalArgumentException("Unexpected argument type: " + arg.getClass().getSimpleName());
			}
		}
		// Assign parsed values
		this.poseSupplier = poseSupplier;
		this.pathConstraints = pathConstraints;
		this.isAuto = isAuto;
		this.currentPoseSupplier = currentPoseSupplier;
		this.tolerance = newTolerance;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		// Reset all controllers
		running = true;
		var currentPose = currentPoseSupplier.get();
		ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
		Translation2d fieldVelocityTranslation = new Translation2d(
				fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
		driveController.reset(
				currentPose.getTranslation()
						.getDistance(poseSupplier.get()),
				Math.min(
						0.0,
						-fieldVelocityTranslation
								.rotateBy(poseSupplier.get()
										.minus(currentPose.getTranslation())
										.getAngle()
										.unaryMinus())
								.getX()));
		updateConstraints(pathConstraints, tolerance);
		lastSetpointTranslation = currentPose.getTranslation();
		RobotContainer.currentPath = "DRIVETOPOSE";
	}

	public void updateConstraints(PathConstraints pathConstraints, double tolerance) {
		this.pathConstraints = pathConstraints;
		this.tolerance = tolerance;
		driveController.setConstraints(new TrapezoidProfile.Constraints(
				pathConstraints.maxVelocityMPS(),
				pathConstraints.maxAccelerationMPSSq()));
		driveController.setTolerance(tolerance);
	}

	@Override
	public void execute() {
		// Update from tunable numbers
		if (driveMaxVelocitySlow.hasChanged(hashCode())
				|| driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode()) || driveKi.hasChanged(hashCode())) {
			driveController.setP(driveKp.get());
			driveController.setD(driveKd.get());
			driveController.setI(driveKi.get());
			driveController.setConstraints(new TrapezoidProfile.Constraints(
					pathConstraints.maxVelocityMPS(),
					pathConstraints.maxAccelerationMPSSq()));
		}
		RobotContainer.currentPath = "DRIVETOPOSE";
		// Get current and target pose
		var currentPose = currentPoseSupplier.get();
		//look ahwead
		//currentPose = currentPose.exp(drive.getChassisSpeeds().toTwist2d(.3));
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
		lastSetpointTranslation = new Pose2d(
				targetPose,
				new Rotation2d(
						Math.atan2(
								currentPose.getTranslation().getY() - targetPose.getY(),
								currentPose.getTranslation().getX() - targetPose.getX())))
				.transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
				.getTranslation();
		Translation2d driveVelocity = new Pose2d(
				Translation2d.kZero,
				new Rotation2d(
						Math.atan2(
								currentPose.getTranslation().getY() - targetPose.getY(),
								currentPose.getTranslation().getX() - targetPose.getX())))
				.transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
				.getTranslation();
		if (!isAuto) {
			final double linearS = linearFF.get().getNorm();
			driveVelocity = driveVelocity.interpolate(linearFF.get().times(DriveConstants.kMaxSpeedMetersPerSecond),
					linearS);
		}
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				driveVelocity.getX(), driveVelocity.getY(), RobotContainer.angularSpeed,
				currentPose.getRotation());
		chassisSpeeds = GeomUtil.avoidRobots(chassisSpeeds);
		drive.setChassisSpeeds(chassisSpeeds); // assert that we are relative to the current pose
		// Log data
		Logger.recordOutput("DriveToPose/DistanceError", currentDistance);
		Logger.recordOutput("DriveToPose/DistanceSetpoint",
				driveController.getSetpoint().position);
		Logger.recordOutput("Odometry/DriveToPoseSetpoint",
				new Pose2d(lastSetpointTranslation,
						currentPose.getRotation()));
		Logger.recordOutput("Odometry/DriveToPoseGoal", new Pose2d(targetPose, currentPose.getRotation()));
		if (atGoal())
			running = false; // If we've reached our goal, stop command.
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angularSpeed = 0;

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