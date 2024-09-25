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
import frc.robot.utils.drive.DriveConstants;

import java.util.Optional;
import java.util.function.Supplier;

public class AimToRotation extends Command {
	private final DrivetrainS drive;
	private final Supplier<Rotation2d> angleSupplier;
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	// Allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber(
			"AimToRotation/ThetaKp");
	private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber(
			"AimToRotation/ThetaKd");
	private static final LoggableTunedNumber thetaMaxVelocitySlow = new LoggableTunedNumber(
			"AimToRotation/ThetaMaxVelocitySlow");
	private static final LoggableTunedNumber thetaTolerance = new LoggableTunedNumber(
			"AimToRotation/ThetaTolerance");
	// Default the TunedNumbers on boot
	static {
		thetaKp.initDefault(3);
		thetaKd.initDefault(0.0);
		thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(360.0));
		thetaTolerance.initDefault(Units.degreesToRadians(2.0));
	}

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
		if (thetaKp.hasChanged(hashCode()) || thetaKd.hasChanged(hashCode())
				|| thetaMaxVelocitySlow.hasChanged(hashCode())
				|| thetaTolerance.hasChanged(hashCode())) {
			thetaController.setP(thetaKp.get());
			thetaController.setD(thetaKd.get());
			thetaController.setConstraints(new TrapezoidProfile.Constraints(
					thetaMaxVelocitySlow.get(), Double.POSITIVE_INFINITY));
			thetaController.setTolerance(thetaTolerance.get());
		}
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
		drive.changeDeadband(DriveConstants.TrainConstants.kDeadband); // Go back to normal deadband
		drive.stopModules();
	}

	@Override
	public boolean isFinished() { return false; }
}
