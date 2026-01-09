package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class AimToPose extends Command {
	private final DrivetrainS drive;
	private final Supplier<Pose2d> poseSupplier;
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	// Allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber(
			"AimToPose/ThetaKp", 2.5, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber(
			"AimToPose/ThetaKd", 0, TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber thetaMaxVelocitySlow = new LoggableTunedNumber(
			"AimToPose/ThetaMaxVelocitySlow", Units.degreesToRadians(180.0), TuningConstants.isTuningMacros);
	private static final LoggableTunedNumber thetaTolerance = new LoggableTunedNumber(
			"AimToPose/ThetaTolerance", Units.degreesToRadians(1.0), TuningConstants.isTuningMacros);

	/** Aims to the specified pose under full software control. */
	public AimToPose(DrivetrainS drive, Pose2d pose) {
		this(drive, () -> pose);
	}

	/** Aims to the specified pose under full software control. */
	public AimToPose(DrivetrainS drive, Supplier<Pose2d> poseSupplier) {
		this.drive = drive;
		this.poseSupplier = poseSupplier;
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		// Reset the controller
		Pose2d currentPose = drive.getLookAheadPose();
		thetaController.reset(currentPose.getRotation().getRadians(),
				drive.getRotation2d().getRadians());
		drive.changeDeadband(.02); // Make sure the commands aren't trying to move tiny movements when the drivetrain won't allow it
		RobotContainer.currentPath = "AIMTOPOSE";
	}

	@Override
	public void execute() {
		// Update from tunable numbers
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			thetaController.setP(thetaKp.get());
			thetaController.setD(thetaKd.get());
			thetaController.setConstraints(new TrapezoidProfile.Constraints(
					thetaMaxVelocitySlow.get(), 25));
			thetaController.setTolerance(thetaTolerance.get());
		}, thetaKp, thetaKd, thetaMaxVelocitySlow, thetaTolerance);
		RobotContainer.currentPath = "AIMTOPOSE";
		//set Chassis to be aimed at it.
		double targetAngle = GeomUtil.closerAngleToZero(GeomUtil
				.rotationFromCurrentToTarget(drive.getLookAheadPose().getTranslation(),
						poseSupplier.get().getTranslation(),
						GeomUtil.ApproachDirection.BACK));
		Logger.recordOutput("CurretP", thetaController.getP());
		Rotation2d currentRotation = drive.getLookAheadPose().getRotation();
		Logger.recordOutput("TargetAngle", targetAngle);
		Logger.recordOutput("currentROtation", currentRotation);
		RobotContainer.angleOverrider = Optional.of(new Rotation2d(targetAngle));
		double thetaVelocity = thetaController.getSetpoint().velocity
				+ thetaController.calculate(currentRotation.getRadians(),
						targetAngle); //Go to target rotation using FF.
		Logger.recordOutput("THETA", thetaVelocity);
		PPHolonomicDriveController.overrideRotationFeedback(() -> thetaVelocity);
		RobotContainer.angularSpeed = thetaVelocity;
		
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angleOverrider = Optional.empty();
		PPHolonomicDriveController.clearRotationFeedbackOverride();
		RobotContainer.angularSpeed = 0;
		drive.changeDeadband(DriveConstants.DriverConstants.kDeadband); // Go back to normal deadband
		//drive.stopModules();
	}

	@Override
	public boolean isFinished() { return false; }
}
