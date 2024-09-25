package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;

public class BranchAuto extends Command {
	private boolean isFinished = false;
	private ParallelCommandGroup commandGroup = new ParallelCommandGroup();
	private final Pose2d backupPose;
	private final String choreoTraj;
	private final double endSpeed;

	public BranchAuto(String choreoTraj, Pose2d backupPose, double endSpeed) {
		this.choreoTraj = choreoTraj;
		this.endSpeed = endSpeed;
		this.backupPose = backupPose;
	}

	public void initialize() {
		isFinished = false;
		if (RobotContainer.currentGamePieceStatus == RobotContainer.GamePieceState.HAS_NOTE) {
			//PathPlannerPath.fromChoreoTrajectory will automatically execute any event markers in the choreo Traj.
			Command path = AutoBuilder
					.followPath(PathPlannerPath.fromChoreoTrajectory(choreoTraj))
					/*.andThen(PathFinder.goToPose(backupPose,
							() -> DriveConstants.pathConstraints,
							RobotContainer.drivetrainS, true, endSpeed))*/;
			commandGroup = new ParallelCommandGroup(path); //insert custom thing here
			RobotContainer.currentPath = choreoTraj;
		} else {
			RobotContainer.currentPath = "backup" + choreoTraj;
			Command path = PathFinder.goToPose(backupPose,
					() -> DriveConstants.pathConstraints, RobotContainer.drivetrainS,
					true, endSpeed);
			commandGroup = new ParallelCommandGroup(path);
		}
		commandGroup.initialize();
	}

	@Override
	public void execute() {
		if (commandGroup != null) {
			if (commandGroup.isFinished()) {
				System.out.println("BranchAuto is finished");
				commandGroup.end(false);
				isFinished = true;
			}else{
				commandGroup.execute();
			}
		} else {
			//We've lost all sense of time. End the command.
			System.out.println("BranchAuto is finished BAD");
			isFinished = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (commandGroup != null) {
			commandGroup.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
