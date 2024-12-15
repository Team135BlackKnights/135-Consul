package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveToTargetUsingDriveAndAimAtPose;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.AITargets;

public class BranchAuto extends Command {
	private boolean isFinished = false;
	private SequentialCommandGroup commandGroup = new SequentialCommandGroup();
	private final Pose2d nextGamePiece;
	private final double endSpeed;
	private final DrivetrainS drive;
	private final double delayBotAborterBeforeIntake, delayBotAborterToNextPose;
	private final boolean useChoreo;
	/**
	 * Intake, Score, and go to next game piece, all dynamically, and with vision
	 * detecting other robots/field elements.
	 * 
	 * @param drive
	 * @param nextGamePiece               Pose2d of the next game piece to go to
	 * @param endSpeed                    Speed to end the path at (m/s) usually 2-3
	 *                                    m/s
	 * @param delayBotAborterBeforeIntake Delay before the bot aborter is run WHILE
	 *                                    intaking. (usually very short)
	 * @param delayBotAborterToNextPose   Delay before the bot aborter is run AFTER
	 *                                    scoring. (usually longer)
	 */
	public BranchAuto(DrivetrainS drive, Pose2d nextGamePiece, double endSpeed, double delayBotAborterBeforeIntake,
			double delayBotAborterToNextPose, boolean useChoreo) {
		this.drive = drive;
		this.endSpeed = endSpeed;
		this.nextGamePiece = nextGamePiece;
		this.delayBotAborterBeforeIntake = delayBotAborterBeforeIntake;
		this.delayBotAborterToNextPose = delayBotAborterToNextPose;
		this.useChoreo = useChoreo;
	}

	int counter = 0;
	double lastTx = 0, lastTy = 0;
	boolean noteDetected = false;

	private Translation2d updateNotePose() {
		double gamePieceTx = 0, gamePieceTy = 0;
		Pose2d currentPose = drive.getPose();
		if (Constants.currentMode == Mode.SIM) {
			// In simulation, get the current pose, and set the degree value to
			Translation2d targetPieceLocation = RobotContainer.fieldSimulation
					.getClosestGamePieceOnGround().getPose3d().toPose2d()
					.getTranslation();
			Logger.recordOutput("ClosestGamePiece", targetPieceLocation);
			double deltaX = targetPieceLocation.getX() - currentPose.getX();
			double deltaY = targetPieceLocation.getY() - currentPose.getY();
			gamePieceTx = Units.radiansToDegrees(Math.atan2(deltaY, deltaX)); // Use atan2 instead of atan
			gamePieceTx -= currentPose.getRotation().getDegrees();
			gamePieceTx = GeomUtil.closerAngleToZero(Rotation2d.fromDegrees(gamePieceTx));
			double d = currentPose.getTranslation()
					.getDistance(targetPieceLocation);
			double tyRad = Math.PI
					- Units.degreesToRadians(
							VisionConstants.limeLightAngleOffsetDegrees)
					- (Math.PI * 0.5D - Math.atan(d / Units.inchesToMeters(
							VisionConstants.limelightLensHeightoffFloorInches)));
			gamePieceTy = Units.radiansToDegrees(tyRad);
			noteDetected = true;
		} else {
			// THESE ARE IN D E G R E E S
			LimelightHelpers.LimelightTarget_Detector[] results = LimelightHelpers
					.getLatestResults(
							VisionConstants.limelightName).targetingResults.targets_Detector;
			System.out.println(results);
			for (LimelightHelpers.LimelightTarget_Detector object : results) {
				if (object.confidence < .4) {
					continue;
				}
				if (object.classID == AITargets.kGamePiece.getValue()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					noteDetected = true;
					Logger.recordOutput("Vision/NoteDetected", true);
				} else {
					noteDetected = false;
					Logger.recordOutput("Vision/NoteDetected", false);
				}
			}
		}
		if (gamePieceTx == lastTx && gamePieceTy == lastTy) {
			counter++;
		} else {
			counter = 0;
		}
		if (counter > 10) { // prevent flickering of note detection
			noteDetected = false;
		}
		if (noteDetected)
			return GeomUtil
					.calculateFieldRelativePose3d(currentPose, gamePieceTx, gamePieceTy,
							Units.inchesToMeters(
									VisionConstants.limelightLensHeightoffFloorInches),
							Units.inchesToMeters(2),
							VisionConstants.limeLightAngleOffsetDegrees)
					.getTranslation().toTranslation2d();
		return null;
	}

	private SequentialCommandGroup botAbortWithDelay(double delay) {
		return new SequentialCommandGroup(new WaitCommand(delay), new BotAborter(drive));
	}
	public void initialize() {
		isFinished = false;
		// PathPlannerPath.fromChoreoTrajectory will automatically execute any event
		// markers in the choreo Traj.
		commandGroup = new SequentialCommandGroup(
				new ParallelRaceGroup(
						new DriveToTargetUsingDriveAndAimAtPose(drive, this::updateNotePose,
								RobotContainer.visionS::objectVisionOkay, () -> true), //use a sensor to determine if gamempiece detected
						botAbortWithDelay(delayBotAborterBeforeIntake)),
				new ConditionalCommand(// finished intaking, go score the game piece if we have it
						new InstantCommand(() -> {
							if (drive instanceof Swerve){
								((Swerve) drive).pathplannerIndex = 0;
							}
							RobotContainer.currentPath = "Scoring_" + this.getName();
						}).andThen(
								new PoseBreakoff(drive, PoseBreakoff.BreakoffType.shoot,
										DriveConstants.pathConstraints,
										endSpeed,
										!useChoreo).andThen( // finished scoring, go to next game piece
										new InstantCommand(() -> {
											if (drive instanceof Swerve){
												((Swerve) drive).pathplannerIndex = 0;
											}
										})).andThen(
												new ParallelRaceGroup(
														PathFinder.goToPose(nextGamePiece,
																() -> DriveConstants.pathConstraints, drive, true,
																endSpeed),
														botAbortWithDelay(delayBotAborterToNextPose)))),
						new InstantCommand(() -> {
							if (drive instanceof Swerve){
								((Swerve) drive).pathplannerIndex = 0;
							}
							RobotContainer.currentPath = "SkipScoring_" + this.getName();
						}).andThen(
								new ParallelRaceGroup( // finished skipping scoring, go to next game piece
										PathFinder.goToPose(nextGamePiece,
												() -> DriveConstants.pathConstraints, drive, true,
												endSpeed),
										botAbortWithDelay(delayBotAborterToNextPose))),
						() -> RobotContainer.currentGamePieceStatus == RobotContainer.GamePieceState.HAS_NOTE));

		RobotContainer.currentPath = "Intaking_" + this.getName();
		commandGroup.initialize();
	}

	@Override
	public void execute() {
		if (commandGroup != null) {
			if (commandGroup.isFinished()) {
				System.out.println("BranchAuto is finished");
				commandGroup.end(false);
				isFinished = true;
			} else {
				commandGroup.execute();
			}
		} else {
			// We've lost all sense of time. End the command.
			System.out.println("BranchAuto is finished without a proper isFinished.");
			isFinished = false;
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (commandGroup != null) {
			commandGroup.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}
