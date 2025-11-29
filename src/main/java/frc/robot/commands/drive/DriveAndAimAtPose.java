package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;

/**
 * Aims AT a pose, while driving to it. Only takes a
 */
public class DriveAndAimAtPose extends Command {
	private final DrivetrainS drive;
	private final Supplier<Translation2d> poseSupplier;
	private final ProfiledPIDController driveController;
	private final ProfiledPIDController thetaController;
	private boolean isFinished = false;
	private Translation2d lastSetpointTranslation;
	private double driveErrorAbs;
	private final boolean overrideUserControl;
	private final LoggableTunedNumber 
	driveKp = new LoggableTunedNumber("AimToPose/driveKp", 5, TuningConstants.isTuningMacros), 
	driveKd = new LoggableTunedNumber("AimToPose/driveKd", 3, TuningConstants.isTuningMacros), 
	driveTolerance = new LoggableTunedNumber("AimToPose/driveTolerance", .015, TuningConstants.isTuningMacros), 
	maxThetaSpeed = new LoggableTunedNumber("AimToPose/maxThetaSpeed", Math.PI*2, TuningConstants.isTuningMacros), 
	thetaKp = new LoggableTunedNumber("AimToPose/thetaKp", 5, TuningConstants.isTuningMacros), 
	thetaKd = new LoggableTunedNumber("AimToPose/thetaKp", 5, TuningConstants.isTuningMacros), 
	thetaTolerance = new LoggableTunedNumber("AimToPose/thetaTolerance", Units.degreesToRadians(1), TuningConstants.isTuningMacros), 
	ffMaxRadius = new LoggableTunedNumber("AimToPose/ffMaxRadius", 5, TuningConstants.isTuningMacros), 
	ffMinRadius = new LoggableTunedNumber("AimToPose/ffMinRadius", 2, TuningConstants.isTuningMacros); 
	
	public DriveAndAimAtPose(DrivetrainS drive,
			Supplier<Translation2d> poseSupplier, double givenMaxVelocity,
			boolean overrideUserControl) {
		this.drive = drive;
		this.poseSupplier = poseSupplier;
		this.driveController = new ProfiledPIDController(0.0, 0.0, 0.0,
				new TrapezoidProfile.Constraints(givenMaxVelocity,
						DriveConstants.maxTranslationalAcceleration.get()));
		this.thetaController = new ProfiledPIDController(0.0, 0.0, 0.0,
				new TrapezoidProfile.Constraints(maxThetaSpeed.get(),
						DriveConstants.maxRotationalAcceleration.get()));
		this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
		this.overrideUserControl = overrideUserControl;
		addRequirements(drive);
	}

	public void updateConstraints(double givenMaxVelocity) {
		driveController
				.setConstraints(new TrapezoidProfile.Constraints(givenMaxVelocity,
						DriveConstants.maxTranslationalAcceleration.get()));
	}

	@Override
	public void initialize() {
		var currentPose = drive.getLookAheadPose();
		driveController.reset(
				currentPose.getTranslation().getDistance(poseSupplier.get()),
				Math.min( //get our CURRENT speed, and rotate it by our actual position.
						0.0,
						-new Translation2d(drive.getFieldVelocity().dx,
								drive.getFieldVelocity().dy)
										.rotateBy(poseSupplier.get()
												.minus(drive.getLookAheadPose().getTranslation())
												.getAngle().unaryMinus())
										.getX()));
		lastSetpointTranslation = drive.getLookAheadPose().getTranslation();
		thetaController.reset(currentPose.getRotation().getRadians(),
				drive.getRotation2d().getRadians());
		if (overrideUserControl)
			RobotContainer.userDrive = false; //stop user control
		isFinished = false;
	}

	@Override
	public void execute() {
		var currentPose = drive.getLookAheadPose();
		var targetPose = poseSupplier.get();
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			driveController.setP(driveKp.get());
			driveController.setD(driveKd.get());
			driveController.setTolerance(driveTolerance.get());
			thetaController.setP(thetaKp.get());
			thetaController.setD(thetaKd.get());
			thetaController.setTolerance(thetaTolerance.get());
		}, driveKp, driveKd, driveTolerance, thetaKp, thetaKd, thetaTolerance);
		double currentDistance = currentPose.getTranslation()
				.getDistance(poseSupplier.get());
		//how fast should we be moving relative to distance? use circles based off relative distances to figure that out. 
		double ffScaler = MathUtil.clamp((currentDistance - ffMinRadius.get())
				/ (ffMaxRadius.get() - ffMinRadius.get()), 0.0, 1.0);
		driveErrorAbs = currentDistance;
		driveController.reset(lastSetpointTranslation.getDistance(targetPose),
				driveController.getSetpoint().velocity);
		double driveVelocityScalar = driveController.getSetpoint().velocity
				* ffScaler + driveController.calculate(driveErrorAbs, 0.0); //Go to error of zero from wanted pose, using ff
		if (currentDistance < driveController.getPositionTolerance())
			driveVelocityScalar = 0.0; //if there, STOP.
		lastSetpointTranslation = new Pose2d(targetPose,
				currentPose.getTranslation().minus(targetPose).getAngle())
						.transformBy(GeomUtil.translationToTransform(
								driveController.getSetpoint().position, 0.0))
						.getTranslation();
		double targetAngle = GeomUtil.closerAngleToZero(GeomUtil
				.rotationFromCurrentToTarget(currentPose.getTranslation(),
						poseSupplier.get(), GeomUtil.ApproachDirection.FRONT));
		//targetAngle += Units.degreesToRadians(VisionConstants.DriveToAITargetKError.get()); //Add/subtract from this for any tweaking from where camera placed for actual robot error
		Rotation2d currentRotation = currentPose.getRotation();
		Logger.recordOutput("RotateAndDriveToPose/TargetAngle", targetAngle);
		Logger.recordOutput("RotateAndDriveToPose/currentROtation",
				currentRotation);
		RobotContainer.angleOverrider = Optional.of(new Rotation2d(targetAngle));
		double thetaVelocity = thetaController.getSetpoint().velocity
				+ thetaController.calculate(currentRotation.getRadians(),
						targetAngle); //Go to target rotation using FF.
		// Set the chassis speeds
		// Command speeds
		var driveVelocity = new Pose2d(new Translation2d(),
				currentPose.getTranslation().minus(targetPose).getAngle())
						.transformBy(GeomUtil
								.translationToTransform(driveVelocityScalar, 0.0))
						.getTranslation(); //Calculate X and Y speeds from driveVelocity scalar.
		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(),
				driveVelocity.getY(), thetaVelocity, currentPose.getRotation());
		drive.setChassisSpeeds(speeds); //assert that we are relative to the current pose
		// Log data for debugging
		Logger.recordOutput("RotateAndDriveToPose/DriveError", driveErrorAbs);
		Logger.recordOutput("RotateAndDriveToPose/DriveSpeed",
				driveVelocityScalar);
		Logger.recordOutput("RotateAndDriveToPose/ThetaError",
				thetaController.getPositionError());
		Logger.recordOutput("RotateAndDriveToPose/ThetaSpeed", thetaVelocity);
		// Check if both drive and rotation are at their goals
		if (driveController.atGoal() && thetaController.atGoal()) {
			isFinished = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("DriveAndAimToPose ended");
		//force angle rider to be empty
		RobotContainer.angleOverrider = Optional.empty();
		if (overrideUserControl)
			RobotContainer.userDrive = true; //give user control
		drive.stopModules();
	}

	@Override
	public boolean isFinished() { return isFinished; }
}