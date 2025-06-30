package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
	private final Supplier<Double> toleranceSupplier;
	private AimToRotation thetaControllerCommand;
	private DriveToTranslation driveControllerCommand;
	private boolean isFinished = false;
	private final boolean isAuto;
	private PathConstraints constraints;
	private double lastTolerance = 0;
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
		Supplier<Double> toleranceSupplier = () -> Units.inchesToMeters(.75);
		boolean isAuto = false;
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
				} else if (supplier.get() instanceof Double){
					toleranceSupplier = () -> (Double) supplier.get();
				} else if (supplier.get() instanceof PathConstraints){
					pathConstraints =(PathConstraints) supplier.get();
				}
				
				else {
					throw new IllegalArgumentException("Unexpected supplier type: " + supplier.get().getClass().getSimpleName());
				}
			} else if (arg instanceof Translation2d translation) {
				position = () -> translation;
			} else if (arg instanceof Rotation2d rot) {
				rotation = () -> rot;
			} else if (arg instanceof PathConstraints constraints) {
				pathConstraints = constraints;
			} else if (arg instanceof Boolean slowMode) {
				isAuto = slowMode;
			} else if (arg instanceof Double translationalTolerance) {
				toleranceSupplier = () -> translationalTolerance;
			}
			 else {
				throw new IllegalArgumentException("Unexpected argument type: " + arg.getClass().getSimpleName());
			}
		}

		// Assign parsed values

		this.constraints = pathConstraints;
		this.isAuto = isAuto;
		this.toleranceSupplier = toleranceSupplier;
		this.positionSupplier = position;
		this.rotationSupplier = rotation;
		// Don't require the drive system as the translation controller will handle it
	}

	public void updateConstraints(PathConstraints constraints) {
		this.constraints = constraints;
		driveControllerCommand.updateConstraints(constraints, toleranceSupplier.get());
		thetaControllerCommand.updateConstraints(constraints);
	}
	public void updateConstraints(PathConstraints constraints, double tolerance) {
		this.constraints = constraints;
		driveControllerCommand.updateConstraints(constraints, tolerance);
		thetaControllerCommand.updateConstraints(constraints);
	}
	@Override
	public void initialize() {
		isFinished = false;
		RobotContainer.userDrive = false;
		thetaControllerCommand = new AimToRotation(rotationSupplier, drive, constraints);

		System.out.println("Branch score location not provided");
		driveControllerCommand = new DriveToTranslation(drive, isAuto, positionSupplier, constraints);
		thetaControllerCommand.initialize();
		driveControllerCommand.initialize();
		System.out.println("DriveAndAimToRotation initialized");
	}


	@Override
	public void execute() {
		if ((driveControllerCommand.atGoal())
				&& (thetaControllerCommand.atGoal())) {
			isFinished = true;
		} else {
			if (toleranceSupplier.get() != lastTolerance){
				updateConstraints(constraints);
				lastTolerance = toleranceSupplier.get();
			}
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
		if (!interrupted) {
			System.out.println("DriveAndAimToRotation finished");
		}
		drive.stopModules();
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}