package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.LoggableTunedNumber;
import java.util.Optional;
import java.util.function.Supplier;

public class AimToRotation extends Command {
	private final DrivetrainS drive;
	private final Supplier<Rotation2d> angleSupplier;
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	// Allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber(
			"AimToRotation/ThetaKp", 3);
	private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber(
			"AimToRotation/ThetaKd", 0);
	private static final LoggableTunedNumber thetaMaxVelocitySlow = new LoggableTunedNumber(
			"AimToRotation/ThetaMaxVelocitySlow", Units.degreesToRadians(360.0));
	private static final LoggableTunedNumber thetaTolerance = new LoggableTunedNumber(
			"AimToRotation/ThetaTolerance", Units.degreesToRadians(2.0));

	/** Aims to the specified pose under full software control. */
	public AimToRotation(DrivetrainS drive, Rotation2d angle) {
		this(drive, () -> angle);
	}

	/** Aims to the specified pose under full software control. */
	public AimToRotation(DrivetrainS drive, Supplier<Rotation2d> angleSupplier) {
		this.drive = drive;
		this.angleSupplier = angleSupplier;
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		// Reset the controller
		Pose2d currentPose = drive.getPose();
		thetaController.reset(currentPose.getRotation().getRadians(),
				drive.getRotation2d().getRadians());
		drive.changeDeadband(.01); // Make sure the commands aren't trying to move tiny movements when the drivetrain won't allow it
		RobotContainer.currentPath = "AIMTOROTATION";
	}

	@Override
	public void execute() {
		// Update from tunable numbers
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			thetaController.setP(thetaKp.get());
			thetaController.setD(thetaKd.get());
			thetaController.setConstraints(new TrapezoidProfile.Constraints(
					thetaMaxVelocitySlow.get(), Double.POSITIVE_INFINITY));
			thetaController.setTolerance(thetaTolerance.get());
		}, thetaKp, thetaKd, thetaMaxVelocitySlow, thetaTolerance);
		RobotContainer.currentPath = "AIMTOROTATION";
		Rotation2d currentRotation = drive.getPose().getRotation();
		RobotContainer.angleOverrider = Optional.of(angleSupplier.get());
		double thetaVelocity = thetaController.getSetpoint().velocity
				+ thetaController.calculate(currentRotation.getRadians(),
						angleSupplier.get().getRadians()); //Go to target rotation using FF.
		if (Constants.currentMatchState == Constants.FRCMatchState.TELEOP) {
			RobotContainer.angularSpeed = thetaVelocity;
		}
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angleOverrider = Optional.empty();
		RobotContainer.angularSpeed = 0;
		drive.changeDeadband(.1); // Go back to normal deadband
		drive.stopModules();
	}

	@Override
	public boolean isFinished() { return false; }
}
