package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.GeomUtil.ApproachDirection;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.GeomUtil;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

public class AimToRotation extends Command {
	private static final LoggableTunedNumber kP = new LoggableTunedNumber("HeadingController/kP", 10);
	private static final LoggableTunedNumber kD = new LoggableTunedNumber("HeadingController/kD", 0);
	private static final LoggableTunedNumber toleranceDegrees = new LoggableTunedNumber(
			"HeadingController/ToleranceDegrees", 1.0);

	private final ProfiledPIDController controller;
	private final Supplier<Rotation2d> goalHeadingSupplier;

	private final DrivetrainS drive;
	/**
	 * Aim the robot at a specific pose2d
	 * @param goalPose
	 * @param approachDirection
	 * @param drive
	 */
	public AimToRotation(Supplier<Pose2d> goalPose, ApproachDirection approachDirection, DrivetrainS drive) {
		this(() -> GeomUtil.rotationFromCurrentToTarget(
			drive.getPose().getTranslation(),
			goalPose.get().getTranslation(), // Fixed: Call goalPose.get() to retrieve the Pose2d
			approachDirection
		), drive,DriveConstants.pathConstraints);
	}
	/**
	 * Aim the robot at a specific pose2d
	 * @param goalPose
	 * @param drive
	 */
	public AimToRotation(Pose2d goalPose, ApproachDirection approachDirection, DrivetrainS drive) {
		this(() -> GeomUtil.rotationFromCurrentToTarget(
			drive.getPose().getTranslation(),
			goalPose.getTranslation(),
			approachDirection
		), drive, DriveConstants.pathConstraints);
	}
	/**
	 * Aim the robot at a specific heading (tell the robot to go to x rotation)
	 * @param goalHeading
	 * @param drive
	 */
	public AimToRotation(Rotation2d goalHeading, DrivetrainS drive) {
		this(() -> goalHeading, drive, DriveConstants.pathConstraints);
	}
	/**
	 * Aim the robot at a specific heading (tell the robot to go to x rotation)
	 * @param goalHeading
	 * @param drive
	 * @param constraints
	 */
	public AimToRotation(Supplier<Rotation2d> goalHeadingSupplier, DrivetrainS drive, PathConstraints constraints) {
		controller = new ProfiledPIDController(
				kP.get(),
				0,
				kD.get(),
				new TrapezoidProfile.Constraints(constraints.maxAngularVelocityRadPerSec(), constraints.maxAngularAccelerationRadPerSecSq()),
				.02);
		controller.enableContinuousInput(-Math.PI, Math.PI);
		controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
		this.goalHeadingSupplier = goalHeadingSupplier;
		drive.changeDeadband(.02);
		this.drive = drive;
		controller.reset(
				drive.getPose().getRotation().getRadians(),
				drive.getFieldVelocity().dtheta);
	}
	public void updateConstraints(PathConstraints constraints) {
		controller.setConstraints(new TrapezoidProfile.Constraints(constraints.maxAngularVelocityRadPerSec(), constraints.maxAngularAccelerationRadPerSecSq()));
	}
	@Override
	public void execute() {
		// Update controller
		controller.setPID(kP.get(), 0, kD.get());
		controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

		var output = controller.calculate(
				drive.getPose().getRotation().getRadians(),
				goalHeadingSupplier.get().getRadians());

		Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
		PPHolonomicDriveController.overrideRotationFeedback(() -> output);
		RobotContainer.angularSpeed = output;
	}

	/** Returns true if within tolerance of aiming at speaker */
	@AutoLogOutput(key = "Drive/HeadingController/AtGoal")
	public boolean atGoal() {
		return controller.atGoal();
	}
	@Override
	public void initialize() {
		RobotContainer.currentPath = "AIMTOROTATION";
	}
	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angleOverrider = Optional.empty();
		RobotContainer.angularSpeed = 0;
		PPHolonomicDriveController.clearRotationFeedbackOverride();
		drive.changeDeadband(DriveConstants.TrainConstants.kDeadband); // Reset deadband to normal
	}
	@Override
	public boolean isFinished() {
		return false;
	}

}