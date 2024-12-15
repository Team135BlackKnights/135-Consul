package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

/**
	 * Initializes the command with flexible arguments.
	 *
	 * @param drive The drivetrain subsystem.
	 * @param args  Flexible arguments for initialization:
	 * 
	 *              <ul>
	 *              <li><b>Position: (pick ONE)</b>
	 *              <ul>
	 *              <li>`Pose2d`: Uses the translation of the pose.</li>
	 *              <li>`Translation2d`: Uses the target translation.</li>
	 *              <li>`Supplier< Pose2d >`: Dynamically provides the pose's
	 *              translation during execution.</li>
	 *              <li>`Supplier< Translation2d >`: Dynamically provides the
	 *              translation during execution.</li>
	 *              </ul>
	 *              </li>
	 *              <li><b>Rotation: (pick ONE)</b>
	 *              <ul>
	 *              <li>`Pose2d`: Uses the rotation of the pose.</li>
	 *              <li>`Rotation2d`: Uses the target rotation.</li>
	 *              <li>`Supplier< Pose2d >`: Dynamically provides the pose's
	 *              rotation
	 *              during execution.</li>
	 *              <li>`Supplier< Rotation2d >`: Dynamically provides the rotation
	 *              during execution.</li>
	 *              </ul>
	 *              </li>
	 *              <li><b>PathConstraints: (optional)</b> Specifies the path
	 *              constraints to follow, otherwise using
	 *              DriveConstants.pathConstraints as default</li>
	 *              <li><b>Boolean: (optional)</b> Enables slow mode if true,
	 *              otherwise using false as default.</li>
	 *              </ul>
	 */
public class DriveAndAimToRotation extends Command {
	private final DrivetrainS drive;
	private final Supplier<Translation2d> positionSupplier;
	private final Supplier<Rotation2d> rotationSupplier;
	private AimToRotation thetaControllerCommand;
	private DriveToTranslation driveControllerCommand;
	private boolean isFinished = false;
	private final boolean slowMode;
	private PathConstraints constraints;

	/**
	 * Initializes the command with flexible arguments.
	 *
	 * @param drive The drivetrain subsystem.
	 * @param args  Flexible arguments for initialization:
	 * 
	 *              <ul>
	 *              <li><b>Position: (pick ONE)</b>
	 *              <ul>
	 *              <li>`Pose2d`: Uses the translation of the pose.</li>
	 *              <li>`Translation2d`: Uses the target translation.</li>
	 *              <li>`Supplier< Pose2d >`: Dynamically provides the pose's
	 *              translation during execution.</li>
	 *              <li>`Supplier< Translation2d >`: Dynamically provides the
	 *              translation during execution.</li>
	 *              </ul>
	 *              </li>
	 *              <li><b>Rotation: (pick ONE)</b>
	 *              <ul>
	 *              <li>`Pose2d`: Uses the rotation of the pose.</li>
	 *              <li>`Rotation2d`: Uses the target rotation.</li>
	 *              <li>`Supplier< Pose2d >`: Dynamically provides the pose's
	 *              rotation
	 *              during execution.</li>
	 *              <li>`Supplier< Rotation2d >`: Dynamically provides the rotation
	 *              during execution.</li>
	 *              </ul>
	 *              </li>
	 *              <li><b>PathConstraints: (optional)</b> Specifies the path
	 *              constraints to follow, otherwise using
	 *              DriveConstants.pathConstraints as default</li>
	 *              <li><b>Boolean: (optional)</b> Enables slow mode if true,
	 *              otherwise using false as default.</li>
	 *              </ul>
	 */
	@SafeVarargs
	public DriveAndAimToRotation(DrivetrainS drive, Object... args) {
		this.drive = drive;

		// Default values
		Supplier<Translation2d> position = () -> new Translation2d();
		Supplier<Rotation2d> rotation = () -> new Rotation2d();
		PathConstraints pathConstraints = DriveConstants.pathConstraints;
		boolean isSlowMode = false;

		// Parse arguments
		for (Object arg : args) {
			if (arg instanceof Pose2d pose) {
				position = () -> pose.getTranslation();
				rotation = () -> pose.getRotation();
			} else if (arg instanceof Supplier<?> supplier) {
				if (supplier.get() instanceof Pose2d) {
					position = () -> ((Pose2d) supplier.get()).getTranslation();
					rotation = () -> ((Pose2d) supplier.get()).getRotation();
				} else if (supplier.get() instanceof Translation2d) {
					position = () -> (Translation2d) supplier.get();
				} else if (supplier.get() instanceof Rotation2d) {
					rotation = () -> (Rotation2d) supplier.get();
				}
			} else if (arg instanceof Translation2d translation) {
				position = () -> translation;
			} else if (arg instanceof Rotation2d rot) {
				rotation = () -> rot;
			} else if (arg instanceof PathConstraints constraints) {
				pathConstraints = constraints;
			} else if (arg instanceof Boolean slowMode) {
				isSlowMode = slowMode;
			} else {
				throw new IllegalArgumentException("Unexpected argument type: " + arg.getClass().getSimpleName());
			}
		}

		// Assign parsed values
		this.positionSupplier = position;
		this.rotationSupplier = rotation;
		this.constraints = pathConstraints;
		this.slowMode = isSlowMode;

		// Don't require the drive system as the translation controller will handle it
	}

	public void updateConstraints(PathConstraints constraints) {
		this.constraints = constraints;
		driveControllerCommand.updateConstraints(constraints);
		thetaControllerCommand.updateConstraints(constraints);
	}

	@Override
	public void initialize() {
		isFinished = false;
		thetaControllerCommand = new AimToRotation(rotationSupplier, drive, constraints);
		driveControllerCommand = new DriveToTranslation(drive, slowMode, positionSupplier, constraints);
		thetaControllerCommand.initialize();
		driveControllerCommand.initialize();
		System.out.println("DriveAndAimToRotation initialized");
	}


	@Override
	public void execute() {
		if ((driveControllerCommand.atGoal())
				&& (thetaControllerCommand.atGoal())) {
			System.out.println("DriveAndAimToRotation finished");
			isFinished = true;
		} else {
			thetaControllerCommand.execute();
			driveControllerCommand.execute();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (thetaControllerCommand != null) {
			thetaControllerCommand.end(interrupted);
		}
		if (driveControllerCommand != null) {
			driveControllerCommand.end(interrupted);
		}
		// force angle rider to be empty
		RobotContainer.angleOverrider = Optional.empty();
		RobotContainer.angularSpeed = 0;
		RobotContainer.userDrive = true;
		drive.stopModules();
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}