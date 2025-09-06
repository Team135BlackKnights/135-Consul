package frc.robot.utils.CompetitionFieldUtils.Simulation;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.ReefHeight;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;

//YEARLYUPDATE: change this field (field bounds/obstacles and scoring locations) to match the year's competition
/**
 * field simulation for 2025 competition
 * 
 */
public class Reefscape2025FieldSimulation extends CompetitionFieldSimulation {
	public Reefscape2025FieldSimulation(AbstractDriveTrainSimulation robot) {
		// Initialize faces
		super(robot, new ReefscapeFieldObstaclesMap());

		FieldConstants.Reef.blueCenterFaces[0] = new Pose2d(
				Units.inchesToMeters(144.003),
				Units.inchesToMeters(158.500),
				Rotation2d.fromDegrees(180));
		FieldConstants.Reef.blueCenterFaces[1] = new Pose2d(
				Units.inchesToMeters(160.373),
				Units.inchesToMeters(186.857),
				Rotation2d.fromDegrees(120));
		FieldConstants.Reef.blueCenterFaces[2] = new Pose2d(
				Units.inchesToMeters(193.116),
				Units.inchesToMeters(186.858),
				Rotation2d.fromDegrees(60));
		FieldConstants.Reef.blueCenterFaces[3] = new Pose2d(
				Units.inchesToMeters(209.489),
				Units.inchesToMeters(158.502),
				Rotation2d.fromDegrees(0));
		FieldConstants.Reef.blueCenterFaces[4] = new Pose2d(
				Units.inchesToMeters(193.118),
				Units.inchesToMeters(130.145),
				Rotation2d.fromDegrees(-60));
		FieldConstants.Reef.blueCenterFaces[5] = new Pose2d(
				Units.inchesToMeters(160.375),
				Units.inchesToMeters(130.144),
				Rotation2d.fromDegrees(-120));
		FieldConstants.Reef.redCenterFaces[0] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[0], true);
		FieldConstants.Reef.redCenterFaces[1] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[1], true);
		FieldConstants.Reef.redCenterFaces[2] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[2], true);
		FieldConstants.Reef.redCenterFaces[3] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[3], true);
		FieldConstants.Reef.redCenterFaces[4] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[4], true);
		FieldConstants.Reef.redCenterFaces[5] = GeomUtil.apply(FieldConstants.Reef.blueCenterFaces[5], true);
		// Initialize branch positions
		for (int face = 0; face < 6; face++) {
			Map<ReefHeight, Pose3d> blueFillRight = new HashMap<>();
			Map<ReefHeight, Pose3d> blueFillLeft = new HashMap<>();
			Map<ReefHeight, Pose3d> redFillRight = new HashMap<>();
			Map<ReefHeight, Pose3d> redFillLeft = new HashMap<>();
			for (var level : ReefHeight.values()) {
				Pose2d bluePoseDirection = new Pose2d(FieldConstants.Reef.blueCenter,
						Rotation2d.fromDegrees(180 - (60 * face)));
				Pose2d redPoseDirection = GeomUtil.apply(bluePoseDirection, true);
				double adjustX = Units.inchesToMeters(30.738);
				double adjustY = Units.inchesToMeters(6.469);

				blueFillRight.put(
						level,
						new Pose3d(
								new Translation3d(
										bluePoseDirection
												.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
												.getX(),
										bluePoseDirection
												.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
												.getY(),
										level.height),
								new Rotation3d(
										0,
										level.pitch,
										bluePoseDirection.getRotation().getRadians())));
				blueFillLeft.put(
						level,
						new Pose3d(
								new Translation3d(
										bluePoseDirection
												.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
												.getX(),
										bluePoseDirection
												.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
												.getY(),
										level.height),
								new Rotation3d(
										0,
										level.pitch,
										bluePoseDirection.getRotation().getRadians())));
				redFillRight.put(
						level,
						new Pose3d(
								new Translation3d(
										redPoseDirection
												.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
												.getX(),
										redPoseDirection
												.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
												.getY(),
										level.height),
								new Rotation3d(
										0,
										level.pitch,
										redPoseDirection.getRotation().getRadians())));
				redFillLeft.put(
						level,
						new Pose3d(
								new Translation3d(
										redPoseDirection
												.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
												.getX(),
										redPoseDirection
												.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
												.getY(),
										level.height),
								new Rotation3d(
										0,
										level.pitch,
										redPoseDirection.getRotation().getRadians())));
			}
			FieldConstants.Reef.blueBranchPositions.add((face * 2), blueFillRight);
			FieldConstants.Reef.blueBranchPositions.add((face * 2) + 1, blueFillLeft);
			FieldConstants.Reef.redBranchPositions.add((face * 2), redFillRight);
			FieldConstants.Reef.redBranchPositions.add((face * 2) + 1, redFillLeft);
		}
	}

	@Override
	public void placeGamePiecesOnField(boolean preload) {
		for (Translation2d algaePosition : FieldConstants.ALGAE_BALL_INITIAL_POSITIONS)
			super.addGamePiece(new Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated(
					algaePosition));
		for (Pose3d algaeReefPos : FieldConstants.ALGAE_BALL_REEF_POSITIONS)
			super.addGamePiece(new Reefscape2025FieldObjects.AlgaeBallOnFieldStatic(
				algaeReefPos));
		for (Translation2d coralPosition : FieldConstants.REEFSCAPE_CORAL_INITIAL_POSITIONS)
			super.addGamePiece(new Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated(
					coralPosition));
		if (preload) {
			super.addGamePiece(new Reefscape2025FieldObjects.ReefscapeCoralOnManipulator()
			);
		}
	}

	/**
	 * the obstacles on the 2025 competition field
	 */
	public static final class ReefscapeFieldObstaclesMap
			extends FieldObstaclesMap {
		public ReefscapeFieldObstaclesMap() {
			super();
			// left wall
			super.addBorderLine(new Translation2d(0, Units.inchesToMeters(50.75)),
					new Translation2d(0, FieldConstants.FIELD_HEIGHT - Units.inchesToMeters(50.75)),
					Units.inchesToMeters(78));
			// top left HP
			super.addBorderLine(new Translation2d(0, FieldConstants.FIELD_HEIGHT - Units.inchesToMeters(50.75)),
					new Translation2d(Units.inchesToMeters(67.5), FieldConstants.FIELD_HEIGHT),
					Units.inchesToMeters(78));
			// upper wall
			super.addBorderLine(new Translation2d(Units.inchesToMeters(67.5), FieldConstants.FIELD_HEIGHT),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250),
							FieldConstants.FIELD_HEIGHT),
					Units.inchesToMeters(50));
			// red processor
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250),
					FieldConstants.FIELD_HEIGHT),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250),
							FieldConstants.FIELD_HEIGHT + Units.inchesToMeters(22)),
					Units.inchesToMeters(50));
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250),
					FieldConstants.FIELD_HEIGHT + Units.inchesToMeters(22)),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250 - 30),
							FieldConstants.FIELD_HEIGHT + Units.inchesToMeters(22)),
					Units.inchesToMeters(50));
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250 - 30),
					FieldConstants.FIELD_HEIGHT + Units.inchesToMeters(22)),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250 - 30),
							FieldConstants.FIELD_HEIGHT),
					Units.inchesToMeters(50));
			// upper wall
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(250 - 30),
							FieldConstants.FIELD_HEIGHT),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(67.5),
							FieldConstants.FIELD_HEIGHT),
					Units.inchesToMeters(50));
			// top right HP
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(67.5),
							FieldConstants.FIELD_HEIGHT),
					new Translation2d(FieldConstants.FIELD_WIDTH,
							FieldConstants.FIELD_HEIGHT - Units.inchesToMeters(50.75)),
					Units.inchesToMeters(78));
			// right wall
			super.addBorderLine(
					new Translation2d(FieldConstants.FIELD_WIDTH,
							FieldConstants.FIELD_HEIGHT - Units.inchesToMeters(50.75)),
					new Translation2d(FieldConstants.FIELD_WIDTH, Units.inchesToMeters(50.75)),
					Units.inchesToMeters(78));

			// bottom right HP
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH, Units.inchesToMeters(50.75)),
					new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(67.5), 0),
					Units.inchesToMeters(78));
			// bottom wall
			super.addBorderLine(new Translation2d(FieldConstants.FIELD_WIDTH - Units.inchesToMeters(67.5), 0),
					new Translation2d(Units.inchesToMeters(250), 0),
					Units.inchesToMeters(50));
			// blue processor
			super.addBorderLine(new Translation2d(Units.inchesToMeters(250), 0),
					new Translation2d(Units.inchesToMeters(250), -Units.inchesToMeters(22)),
					Units.inchesToMeters(50));
			super.addBorderLine(new Translation2d(Units.inchesToMeters(250), -Units.inchesToMeters(22)),
					new Translation2d(Units.inchesToMeters(250 - 30), -Units.inchesToMeters(22)),
					Units.inchesToMeters(50));
			super.addBorderLine(new Translation2d(Units.inchesToMeters(250 - 30), -Units.inchesToMeters(22)),
					new Translation2d(Units.inchesToMeters(250 - 30), 0),
					Units.inchesToMeters(50));
			// bottom wall
			super.addBorderLine(new Translation2d(Units.inchesToMeters(250 - 30), 0),
					new Translation2d(Units.inchesToMeters(67.5), 0),
					Units.inchesToMeters(50));
			// bottom left HP
			super.addBorderLine(new Translation2d(Units.inchesToMeters(67.5), 0),
					new Translation2d(0, Units.inchesToMeters(50.75)),
					Units.inchesToMeters(78));
			// barge pillar
			super.addRectangularObstacle(Units.inchesToMeters(12), Units.inchesToMeters(12),
					new Pose2d(new Translation2d(FieldConstants.FIELD_WIDTH / 2, FieldConstants.FIELD_HEIGHT / 2),
							new Rotation2d()),
					0);
			Translation2d[] reefVorticesBlue = new Translation2d[] {
                new Translation2d(3.658, 3.546),
                new Translation2d(3.658, 4.506),
                new Translation2d(4.489, 4.987),
                new Translation2d(5.3213, 4.506),
                new Translation2d(5.3213, 3.546),
                new Translation2d(4.489, 3.065)
            };
            for (int i = 0; i < 6; i++) super.addBorderLine(reefVorticesBlue[i], reefVorticesBlue[(i + 1) % 6],Units.inchesToMeters(17.5));

            // red reef
            Translation2d[] reefVorticesRed = Arrays.stream(reefVorticesBlue)
                    .map(pointAtBlue ->
                            new Translation2d(FieldConstants.FIELD_WIDTH - pointAtBlue.getX(), pointAtBlue.getY()))
                    .toArray(Translation2d[]::new);
            for (int i = 0; i < 6; i++) super.addBorderLine(reefVorticesRed[i], reefVorticesRed[(i + 1) % 6], Units.inchesToMeters(17.5));

			/*
			 * super.addRectangularObstacle(0.35, 0.35,
			 * new Pose2d(FieldConstants.FIELD_WIDTH - 5.62, 4.1 + 1.28,
			 * Rotation2d.fromDegrees(30)),
			 * Units.inchesToMeters(75));
			 */
		}
	}
}
