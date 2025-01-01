package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveToTargetUsingDriveAndAimAtPose;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;

public class BranchAutoSegment extends Command {
	private boolean isFinished = false;
	private SequentialCommandGroup commandGroup = new SequentialCommandGroup();
	private final Pose2d nextGamePiece;
	private final double endSpeed;
	private final DrivetrainS drive;
	private final double delayBotAborterToNextPose;
	private final boolean useChoreo;

	/**
	 * Intake, Score, and go to next game piece, all dynamically, and with vision
	 * detecting other robots/field elements.
	 * 
	 * @param drive
	 * @param nextGamePiece             Pose2d of the next game piece to go to
	 * @param endSpeed                  Speed to end the path at (m/s) usually 2-3
	 *                                  m/s
	 * @param delayBotAborterToNextPose Delay before the bot aborter is run AFTER
	 *                                  scoring. (usually longer)
	 */
	public BranchAutoSegment(DrivetrainS drive, Pose2d nextGamePiece, double endSpeed,
			double delayBotAborterToNextPose, boolean useChoreo) {
		this.drive = drive;
		this.endSpeed = endSpeed;
		this.nextGamePiece = nextGamePiece;
		this.delayBotAborterToNextPose = delayBotAborterToNextPose;
		this.useChoreo = useChoreo;
	}

	private SequentialCommandGroup botAbortWithDelay(double delay) {
		return new SequentialCommandGroup(new WaitCommand(delay), new BotAborter(drive));
	}

	private InstantCommand changePath(String prefix) {
		return new InstantCommand(
				() -> {
					RobotContainer.currentPath = prefix + "_" + this.getName();
				});
	}

	public void initialize() {
		isFinished = false;
		// PathPlannerPath.fromChoreoTrajectory will automatically execute any event
		// markers in the choreo Traj.
		commandGroup = new SequentialCommandGroup(
			new ConditionalCommand(
				// if we didn't abort the drive to gamepiece, then we can go to the next game piece
				changePath("Intaking").andThen(
					new DriveToTargetUsingDriveAndAimAtPose(drive, Vision::updateNotePose,
							RobotContainer.visionS::objectVisionOkay, () -> false))
					.andThen(
						new ConditionalCommand(// finished intaking, go score the game piece if we have it
							changePath("Scoring")
								.andThen(
									new PoseBreakoff(drive, PoseBreakoff.BreakoffType.shoot,
										DriveConstants.pathConstraints,
										endSpeed,
										!useChoreo).andThen( // finished scoring, go to next game piece
											changePath("NextGamePiece"))
										.andThen(
											new ParallelRaceGroup(
													PathFinder.goToPose(
															nextGamePiece,
															() -> DriveConstants.pathConstraints,
															drive,
															true,
															endSpeed),
													botAbortWithDelay(
															delayBotAborterToNextPose)))),
							changePath("SkipScoring").andThen(
								new ParallelRaceGroup( // finished skipping scoring, go to next game piece
										PathFinder.goToPose(nextGamePiece,
												() -> DriveConstants.pathConstraints, drive,
												true,
												endSpeed),
										botAbortWithDelay(delayBotAborterToNextPose))),
							() -> RobotContainer.currentGamePieceStatus == RobotContainer.GamePieceState.HAS_NOTE)),  //use a sensor
				changePath("SkipIntaking").andThen(
						new ParallelRaceGroup( // finished skipping scoring, go to next game piece
								PathFinder.goToPose(nextGamePiece,
										() -> DriveConstants.pathConstraints, drive, true,
										endSpeed),
								botAbortWithDelay(delayBotAborterToNextPose))),
				() -> RobotContainer.currentGamePieceStatus != RobotContainer.GamePieceState.ABORT));

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
