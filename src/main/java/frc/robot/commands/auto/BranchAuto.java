package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;

public class BranchAuto extends Command {
	private boolean isFinished = false;
	private Command path;
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
		if (RobotContainer.currentGamePieceStatus == 0) {
			//PathPlannerPath.fromChoreoTrajectory will automatically execute any event markers in the choreo Traj.
			path = AutoBuilder
					.followPath(PathPlannerPath.fromChoreoTrajectory(choreoTraj))
					.andThen(PathFinder.goToPose(backupPose,
							() -> DriveConstants.pathConstraints,
							RobotContainer.drivetrainS, true, endSpeed));
		} else {
			path = PathFinder.goToPose(backupPose,
					() -> DriveConstants.pathConstraints, RobotContainer.drivetrainS,
					true, endSpeed);
		}
		path.initialize();
	}

	@Override
	public void execute() {
		if (path != null) {
			path.execute();
			if (path.isFinished()) {
				path.end(false);
				isFinished = true;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (path != null) {
			path.end(interrupted);
		}
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
