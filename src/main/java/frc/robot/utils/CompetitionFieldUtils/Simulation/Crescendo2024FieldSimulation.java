package frc.robot.utils.CompetitionFieldUtils.Simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;

/**
 * field simulation for 2024 competition
 */
public class Crescendo2024FieldSimulation extends CompetitionFieldSimulation {
	public Crescendo2024FieldSimulation(HolonomicChassisSimulation robot) {
		super(robot, new CrescendoFieldObstaclesMap());
	}

	@Override
	public void placeGamePiecesOnField(boolean preload) {
		for (Translation2d notePosition : FieldConstants.NOTE_INITIAL_POSITIONS)
			super.addGamePiece(new Crescendo2024FieldObjects.NoteOnFieldSimulated(
					notePosition));
		if (preload) {
			super.addGamePiece(new Crescendo2024FieldObjects.NoteOnManipulator(
					Logger.getTimestamp(), 999,
					super.getMainDriveSimulation().getPose3d(),
					GeometryConstants.launcherTransform));
		}
	}

	/**
	 * the obstacles on the 2024 competition field
	 */
	public static final class CrescendoFieldObstaclesMap
			extends FieldObstaclesMap {
		public CrescendoFieldObstaclesMap() {
			super();
			//left wall
			super.addBorderLine(new Translation2d(0, 1),
					new Translation2d(0, 4.51));
			super.addBorderLine(new Translation2d(0, 4.51),
					new Translation2d(0.9, 5));
			super.addBorderLine(new Translation2d(0.9, 5),
					new Translation2d(0.9, 6.05));
			super.addBorderLine(new Translation2d(0.9, 6.05),
					new Translation2d(0, 6.5));
			super.addBorderLine(new Translation2d(0, 6.5),
					new Translation2d(0, 8.2));
			// upper wall
			super.addBorderLine(new Translation2d(0, 8.12),
					new Translation2d(FieldConstants.FIELD_WIDTH, 8.12));
			// righter wall 
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH, 1),
					new Translation2d(FieldConstants.FIELD_WIDTH, 4.51));
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH, 4.51),
					new Translation2d(FieldConstants.FIELD_WIDTH - 0.9, 5));
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH - 0.9, 5),
					new Translation2d(FieldConstants.FIELD_WIDTH - 0.9, 6.05));
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH - 0.9, 6.05),
					new Translation2d(FieldConstants.FIELD_WIDTH, 6.5));
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH, 6.5),
					new Translation2d(FieldConstants.FIELD_WIDTH, 8.2));
			// lower wall
			super.addBorderLine(new Translation2d(1.92, 0),
					new Translation2d(FieldConstants.FIELD_WIDTH - 1.92, 0));
			// red source wall
			super.addBorderLine(new Translation2d(1.92, 0),
					new Translation2d(0, 1));
			// blue source wall
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH - 1.92, 0),
					new Translation2d(FieldConstants.FIELD_WIDTH, 1));
			// blue stage
			super.addRectangularObstacle(0.35, 0.35,
					new Pose2d(3.4, 4.1, new Rotation2d()));
			super.addRectangularObstacle(0.35, 0.35,
					new Pose2d(5.62, 4.1 - 1.28, Rotation2d.fromDegrees(30)));
			super.addRectangularObstacle(0.35, 0.35,
					new Pose2d(5.62, 4.1 + 1.28, Rotation2d.fromDegrees(60)));
			// red stage
			super.addRectangularObstacle(0.35, 0.35, new Pose2d(
					FieldConstants.FIELD_WIDTH - 3.4, 4.1, new Rotation2d()));
			super.addRectangularObstacle(0.35, 0.35,
					new Pose2d(FieldConstants.FIELD_WIDTH - 5.62, 4.1 - 1.28,
							Rotation2d.fromDegrees(60)));
			super.addRectangularObstacle(0.35, 0.35,
					new Pose2d(FieldConstants.FIELD_WIDTH - 5.62, 4.1 + 1.28,
							Rotation2d.fromDegrees(30)));
		}
	}
}
