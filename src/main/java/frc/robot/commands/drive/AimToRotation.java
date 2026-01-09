package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
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
	private static final LoggableTunedNumber kP = new LoggableTunedNumber("HeadingController/kP", 6, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber kD = new LoggableTunedNumber("HeadingController/kD", .2, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber toleranceDegrees = new LoggableTunedNumber(
			"HeadingController/ToleranceDegrees",2.0, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber toleranceMinSpeed = new LoggableTunedNumber("HeadingController/MinRadPerSec",0.25, TuningConstants.isTuningMacros);
	private final ProfiledPIDController controller;
	private final Supplier<Rotation2d> goalHeadingSupplier;
	private final MedianFilter filter;
	private final DrivetrainS drive;
	private double output = 0;
	/**
	 * Aim the robot at a specific pose2d
	 * @param goalPose
	 * @param approachDirection
	 * @param drive
	 */
	public AimToRotation(Supplier<Pose2d> goalPose, ApproachDirection approachDirection, DrivetrainS drive, PathConstraints constraints) {
		this(() -> GeomUtil.rotationFromCurrentToTarget(
			drive.getLookAheadPose().getTranslation(),
			goalPose.get().getTranslation(), // Fixed: Call goalPose.get() to retrieve the Pose2d
			approachDirection
		), drive,constraints);
	}
	/**
	 * Aim the robot at a specific pose2d
	 * @param goalPose
	 * @param drive
	 */
	public AimToRotation(Pose2d goalPose, ApproachDirection approachDirection, DrivetrainS drive) {
		this(() -> GeomUtil.rotationFromCurrentToTarget(
			drive.getLookAheadPose().getTranslation(),
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
		this.drive = drive;
		this.filter = new MedianFilter(5);
		controller.reset(
				drive.getLookAheadPose().getRotation().getRadians(),
				drive.getFieldVelocity().dtheta);
	}
	public void updateConstraints(PathConstraints constraints) {
		controller.setConstraints(new TrapezoidProfile.Constraints(constraints.maxAngularVelocityRadPerSec(), constraints.maxAngularAccelerationRadPerSecSq()));
	}
	private double getOutput(){
		return output;
	}
	@Override
	public void execute() {
		// Update controller
		controller.setPID(kP.get(), 0, kD.get());
		controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
		double targetRads = filter.calculate(goalHeadingSupplier.get().getRadians());
		output = controller.calculate(
				drive.getLookAheadPose().getRotation().getRadians(),
				targetRads);
		Logger.recordOutput("Drive/HeadingController/HeadingGoal", targetRads);
		Logger.recordOutput("Drive/HeadingController/Output Rad/s Before Deadband", output);
		Logger.recordOutput("Drive/HeadingController/HeadingError", controller.getPositionError());
		if (Math.abs(output) < toleranceMinSpeed.get()){
			output = 0;
		} 
		Logger.recordOutput("Drive/HeadingController/Output Rad/s After Deadband", output);

		PPHolonomicDriveController.overrideRotationFeedback(this::getOutput);
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
	}
	@Override
	public boolean isFinished() {
		return false;
	}

}