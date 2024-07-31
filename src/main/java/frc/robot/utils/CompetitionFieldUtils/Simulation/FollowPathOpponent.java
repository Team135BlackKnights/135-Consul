package frc.robot.utils.CompetitionFieldUtils.Simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

/**
 * this replaces AutoBuilder because in our code, opponent robots are simulated
 * using path-planner drive commands only the main robot should use AutoBuilder,
 * opponent robots should use this
 */
public class FollowPathOpponent extends FollowPathHolonomic {
	public FollowPathOpponent(BooleanSupplier shouldFlipPath,
			DrivetrainS driveSubsystem, double speedMultiplier,
			Rotation2d endingRotation, Pose2d... poses) {
		this(new PathPlannerPath(PathPlannerPath.bezierFromPoses(poses),
				DriveConstants.pathConstraints,
				new GoalEndState(0, endingRotation)), shouldFlipPath,
				driveSubsystem);
	}

	public FollowPathOpponent(PathPlannerPath path,
			BooleanSupplier shouldFlipPath, DrivetrainS driveSubsystem) {
		super(path, driveSubsystem::getPose, driveSubsystem::getChassisSpeeds,
				driveSubsystem::setChassisSpeeds,
				new PIDConstants(
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getP(),
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getI(),
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getD()),
				new PIDConstants(
						DriveConstants.TrainConstants.pathplannerRotationConstantContainer
								.getP(),
						DriveConstants.TrainConstants.pathplannerRotationConstantContainer
								.getI(),
						DriveConstants.TrainConstants.pathplannerRotationConstantContainer
								.getD()),
				DriveConstants.pathConstraints.getMaxVelocityMps(),
				DriveConstants.pathConstraints.getMaxVelocityMps()
						/ DriveConstants.pathConstraints.getMaxAngularVelocityRps(),
				Robot.defaultPeriodSecs, new ReplanningConfig(true, true),
				shouldFlipPath, driveSubsystem);
	}

	public static PathPlannerPath reversePath(PathPlannerPath originalPath,
			GoalEndState endState) {
		final List<PathPoint> newPoints = new ArrayList<>(),
				originalPoints = originalPath.getAllPathPoints();
		for (int i = originalPoints.size() - 1; i >= 0; i--) {
			final PathPoint originalPoint = originalPoints.get(i);
			newPoints.add(new PathPoint(originalPoint.position,
					originalPoint.rotationTarget, originalPoint.constraints));
		}
		return PathPlannerPath.fromPathPoints(newPoints,
				originalPath.getGlobalConstraints(), endState);
	}
}