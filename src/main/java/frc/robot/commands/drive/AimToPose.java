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
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import java.util.Optional;
import java.util.function.Supplier;

public class AimToPose extends Command {
	private final DrivetrainS drive;
	private final Supplier<Pose2d> poseSupplier;
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	// Allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber(
			"AimToPose/ThetaKp");
	private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber(
			"AimToPose/ThetaKd");
	private static final LoggableTunedNumber thetaMaxVelocitySlow = new LoggableTunedNumber(
			"AimToPose/ThetaMaxVelocitySlow");
	private static final LoggableTunedNumber thetaTolerance = new LoggableTunedNumber(
			"AimToPose/ThetaTolerance");
	// Default the TunedNumbers on boot
	static {
		thetaKp.initDefault(7);
		thetaKd.initDefault(0.0);
		thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(180.0));
		thetaTolerance.initDefault(Units.degreesToRadians(4.0));
	}

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
		Pose2d currentPose = drive.getPose();
		thetaController.reset(currentPose.getRotation().getRadians(),
				drive.getRotation2d().getRadians());
		drive.changeDeadband(.01); // Make sure the commands aren't trying to move tiny movements when the drivetrain won't allow it
		RobotContainer.currentPath = "AIMTOPOSE";
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
		RobotContainer.currentPath = "AIMTOPOSE";
		//set Chassis to be aimed at it.
		double targetAngle = GeomUtil
				.rotationFromCurrentToTarget(drive.getPose(), poseSupplier.get(),GeomUtil.ApproachDirection.BACK)
				.getRadians();
		Rotation2d currentRotation = drive.getPose().getRotation();
		RobotContainer.angleOverrider = Optional.of(new Rotation2d(targetAngle));
		double thetaVelocity = thetaController.getSetpoint().velocity
				+ thetaController.calculate(currentRotation.getRadians(),
						targetAngle); //Go to target rotation using FF.
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
