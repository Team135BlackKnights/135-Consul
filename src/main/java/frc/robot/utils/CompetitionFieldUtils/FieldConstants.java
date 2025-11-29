package frc.robot.utils.CompetitionFieldUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.GeomUtil;


/**
 * Constants for the field MUST CREATE A NEW CLASS FOR EACH NEW GAME, @see
 * Reefscape2025FieldSimulation & @see Reefscape2025FieldObjects for example
 */
public class FieldConstants {
	// TOOD: update these
	public static final double FIELD_WIDTH = 17.548;
	public static final double FIELD_HEIGHT = 8.042;
	public static final double// Hexagon points around (130, 140)
	// Hexagon centered at (144 + 32.4, 140 + 37) with a radius of 40 inches
	blueCenterX = Units.inchesToMeters(144 + 32.4),
			blueCenterY = Units.inchesToMeters(140 + 37 / 2),
			radius = Units.inchesToMeters(46),
			redCenterX = FIELD_WIDTH - Units.inchesToMeters(144 + 32.4),
			redCenterY = Units.inchesToMeters(140 + 37 / 2);
	// id 1 is topmost leftmost. goes in order down, right.
	public static final double COEFFICIENT_OF_GRAVITY = 9.81;
	public static final Pose2d START_POSE_LEFT = new Pose2d(7.02, FIELD_HEIGHT-2.5,
			new Rotation2d(-2*Math.PI / 3));
	public static final Pose2d START_POSE_RIGHT = new Pose2d(7.02, 2.5,
			new Rotation2d(Math.PI / 3));
	public static final Translation3d BLUE_SCORING_LOCATION_REEFSCAPE_CORAL = new Translation3d(0.225,
			5.55, 2.1);
	public static final Translation3d RED_SCORING_LOCATION_REEFSCAPE_CORAL = new Translation3d(16.317,
			5.55, 2.1); // in meters!
	public static final double SCORING_COLLISION_RADIUS_REEFSCAPE_CORAL = Units.inchesToMeters(2.25);
	public static final int ALGAE_BALL_SCORE = 6,
			REEFSCAPE_CORAL_SCORE_L1_AUTO = 3,
			REEFSCAPE_CORAL_SCORE_L2_AUTO = 4,
			REEFSCAPE_CORAL_SCORE_L3_AUTO = 6,
			REEFSCAPE_CORAL_SCORE_L4_AUTO = 7,
			REEFSCAPE_CORAL_SCORE_L1_TELE = 2,
			REEFSCAPE_CORAL_SCORE_L2_TELE = 3,
			REEFSCAPE_CORAL_SCORE_L3_TELE = 4,
			REEFSCAPE_CORAL_SCORE_L4_TELE = 5;
	public static final Translation2d[] ALGAE_BALL_INITIAL_POSITIONS = new Translation2d[] {
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5 + 10)),
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5 + 10)),
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5 + 10)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(86.5 + 10)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(158.5 + 10)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(230.5 + 10)),

	};
	public static final Translation2d[] REEFSCAPE_CORAL_INITIAL_POSITIONS = new Translation2d[] {
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5)), // 48, 86.5
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5)),
			new Translation2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(86.5)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(158.5)),
			new Translation2d(FIELD_WIDTH - Units.inchesToMeters(48), Units.inchesToMeters(230.5)),
	};
	public static final Pose3d[] ALGAE_BALL_REEF_POSITIONS = new Pose3d[]{
		new Pose3d(3.81,4.03,1.3, new Rotation3d()),	//1-12
		new Pose3d(4.125,4.625,.9, new Rotation3d()),	//2-3
		new Pose3d(4.85,4.625,1.3, new Rotation3d()),	//4-5
		new Pose3d(5.15,4.03,.9, new Rotation3d()),	//6-7
		new Pose3d(4.85,3.4,1.3, new Rotation3d()),	//8-9
		new Pose3d(4.125,3.4,.9, new Rotation3d()),	//10-11
		new Pose3d(GeomUtil.applyX(3.81, true),GeomUtil.applyY(4.03, true),1.3, new Rotation3d()),
		new Pose3d(GeomUtil.applyX(4.125, true),GeomUtil.applyY(4.625, true),.9, new Rotation3d()),
		new Pose3d(GeomUtil.applyX(4.85, true),GeomUtil.applyY(4.625, true),1.3, new Rotation3d()),
		new Pose3d(GeomUtil.applyX(5.15, true),GeomUtil.applyY(4.03, true),.9, new Rotation3d()),
		new Pose3d(GeomUtil.applyX(4.85, true),GeomUtil.applyY(3.4, true),1.3, new Rotation3d()),
		new Pose3d(GeomUtil.applyX(4.125, true),GeomUtil.applyY(3.4, true),.9, new Rotation3d()),
		
	};
	public static Translation2d getClosestGamePieceFromListOfGamePieces(
			Translation2d robotPosition, Translation2d[] gamePieces) {
		Translation2d closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		for (Translation2d gamePiece : gamePieces) {
			double distance = gamePiece.getDistance(robotPosition);
			if (distance < closestDistance) {
				closestGamePiece = gamePiece;
				closestDistance = distance;
			}
		}
		return closestGamePiece;
	}

	public static enum GamePiece {
		ALGAE_BALL, REEFSCAPE_CORAL
	}

	/* Game Piece 1 Andymark Location */
	public static final double ALGAE_BALL_HEIGHT = Units.inchesToMeters(16); // meters
	public static final double ALGAE_BALL_DIAMETER = Units.inchesToMeters(16); // meters
	// Game Piece 2 Andymark Location
	public static final double REEFSCAPE_CORAL_HEIGHT = Units.inchesToMeters(4.5); // meters
	public static final double REEFSCAPE_CORAL_DIAMETER = Units.inchesToMeters(4.5); // meters
	public static final double REEFSCAPE_CORAL_LENGTH = Units.inchesToMeters(12); // meters

	// Game piece 2 scoring locations:
	public static class Processor {
		public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0,
				Rotation2d.fromDegrees(90));
	}

	public static class Barge {
		public static final Translation2d farCage = new Translation2d(Units.inchesToMeters(345.428),
				Units.inchesToMeters(286.779));
		public static final Translation2d middleCage = new Translation2d(Units.inchesToMeters(345.428),
				Units.inchesToMeters(242.855));
		public static final Translation2d closeCage = new Translation2d(Units.inchesToMeters(345.428),
				Units.inchesToMeters(199.947));

		// Measured from floor to bottom of cage
		public static final double deepHeight = Units.inchesToMeters(3.125);
		public static final double shallowHeight = Units.inchesToMeters(30.125);
	}

	public static class CoralStation {
		public static final Pose2d blueLeftTopFace = new Pose2d(1.6, 7.475, Rotation2d.fromDegrees(90 - 144.011));
		public static final Pose2d blueLeftBottomFace = new Pose2d(.7, 6.7, Rotation2d.fromDegrees(90 - 144.011));
		public static final Pose2d blueLeftCenterFace = new Pose2d(
				Units.inchesToMeters(33.526),
				Units.inchesToMeters(291.176),
				Rotation2d.fromDegrees(90 - 144.011));
		public static final Translation2d[] validBlueLeft = new Translation2d[] {
			new Translation2d(1.57, 7.43),
			new Translation2d(1.37, 7.28),
			new Translation2d(1.28, 7.2),
			new Translation2d(1.11, 7.05),
			new Translation2d(.93, 6.89),
			new Translation2d(.75, 6.74),
			};
		public static final Pose2d blueRightTopFace = new Pose2d(1.57, .6, Rotation2d.fromDegrees(144.011 - 90));
		public static final Pose2d blueRightBottomFace = new Pose2d(.7, 1.45, Rotation2d.fromDegrees(144.011- 90));
		public static final Pose2d blueRightCenterFace = new Pose2d(
				1.117,
				1,
				Rotation2d.fromDegrees(144.011 - 90));
		public static final Translation2d[] validBlueRight = new Translation2d[] {
			new Translation2d(1.57, GeomUtil.applyY(validBlueLeft[0].getY(),true)),
			new Translation2d(1.37, GeomUtil.applyY(validBlueLeft[1].getY(), true)),
			new Translation2d(1.28, GeomUtil.applyY(validBlueLeft[2].getY(), true)),
			new Translation2d(1.11, GeomUtil.applyY(validBlueLeft[3].getY(), true)),
			new Translation2d(.93, GeomUtil.applyY(validBlueLeft[4].getY(), true)),
			new Translation2d(.75, GeomUtil.applyY(validBlueLeft[5].getY(), true)),
			};
		public static final Pose2d redLeftCenterFace = GeomUtil.apply(blueLeftCenterFace, true);
		public static final Pose2d redLeftTopFace = GeomUtil.apply(blueLeftTopFace, true);
		public static final Pose2d redLeftBottomFace = GeomUtil.apply(blueLeftBottomFace, true);
		public static final Translation2d[] validRedLeft = new Translation2d[] {
			GeomUtil.apply(validBlueLeft[0],true),
			GeomUtil.apply(validBlueLeft[1],true),
			GeomUtil.apply(validBlueLeft[2],true),
			GeomUtil.apply(validBlueLeft[3],true),
			GeomUtil.apply(validBlueLeft[4],true),
			GeomUtil.apply(validBlueLeft[5],true),
			};
		public static final Pose2d redRightTopFace = GeomUtil.apply(blueRightTopFace, true);
		public static final Pose2d redRightBottomFace = GeomUtil.apply(blueRightBottomFace, true);
		public static final Pose2d redRightCenterFace = GeomUtil.apply(blueRightCenterFace, true);
		public static final Translation2d[] validRedRight = new Translation2d[] {
			GeomUtil.apply(validBlueRight[0],true),
			GeomUtil.apply(validBlueRight[1],true),
			GeomUtil.apply(validBlueRight[2],true),
			GeomUtil.apply(validBlueRight[3],true),
			GeomUtil.apply(validBlueRight[4],true),
			GeomUtil.apply(validBlueRight[5],true),
			};
	}

	public static class Reef {
		public static final Translation2d blueCenter = new Translation2d(Units.inchesToMeters(176.746),
				Units.inchesToMeters(158.501));
		public static final Translation2d redCenter = GeomUtil.apply(blueCenter);
		public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the
																				// reef zone line

		public static final Pose2d[] blueCenterFaces = new Pose2d[6]; // Starting facing the driver station in clockwise
																		// order
		public static final Pose2d[] redCenterFaces = new Pose2d[6]; // Starting facing the driver station in clockwise
		public static final List<Map<ReefHeight, Pose3d>> blueBranchPositions = new ArrayList<>(36); // Starting at the
																										// right
		// branch facing the
		// driver station in
		// clockwise
		public static final List<Map<ReefHeight, Pose3d>> redBranchPositions = new ArrayList<>(36); // Starting at the
																									// right
	}

	public static class StagingPositions {
		// Measured from the center of the trees
		public static final Pose2d blueLeftTree = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5),
				new Rotation2d());
		public static final Pose2d blueMiddleTree = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5),
				new Rotation2d());
		public static final Pose2d blueRightTree = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5),
				new Rotation2d());
		public static final Pose2d redLeftTree = GeomUtil.apply(blueLeftTree, true);
		public static final Pose2d redMiddleTree = GeomUtil.apply(blueMiddleTree, true);
		public static final Pose2d redRightTree = GeomUtil.apply(blueRightTree, true);
	}

	public static enum ReefHeight {
		L4(Units.inchesToMeters(72), Units.degreesToRadians(-90)),
		L3(Units.inchesToMeters(47.625), Units.degreesToRadians(-35)),
		L2(Units.inchesToMeters(31.875), Units.degreesToRadians(-35)),
		L1(Units.inchesToMeters(18), 0);

		ReefHeight(double height, double pitch) {
			this.height = height;
			this.pitch = pitch; // in degrees
		}

		public final double height;
		public final double pitch;
	}

	public enum GamePieceTag {
		ON_GROUND_ALGAE_BALL, IN_ROBOT_ALGAE_BALL, IN_AIR_ALGAE_BALL, ON_GROUND_REEFSCAPE_CORAL,
		IN_ROBOT_REEFSCAPE_CORAL, IN_AIR_REEFSCAPE_CORAL
	}

	public static final class AlgaeBall {
		public static final double DEFAULT_MASS_KG = Units.lbsToKilograms(1.5), LINEAR_DAMPING = 2.5,
				ANGULAR_DAMPING = 5, EDGE_COEFFICIENT_OF_FRICTION = 0.8,
				EDGE_COEFFICIENT_OF_RESTITUTION = 0.9, ALGAE_COEFFICIENT_OF_AIR_RESISTANCE_K = 1.89, // Empirically
																										// determined.
																										// We should
																										// figure out if
																										// this is
																										// right.,
				GRAVITATIONAL_ACCELERATION_MS2 = .5, M_OVER_K = DEFAULT_MASS_KG / ALGAE_COEFFICIENT_OF_AIR_RESISTANCE_K;
	}

	public static final class ReefscapeCoral {
		public static final double DEFAULT_MASS_KG = Units.lbsToKilograms(1.4), LINEAR_DAMPING = 2.5,
				ANGULAR_DAMPING = 5, EDGE_COEFFICIENT_OF_FRICTION = 0.8,
				EDGE_COEFFICIENT_OF_RESTITUTION = 0.4, COEFFICIENT_OF_AIR_RESISTANCE_K = .3,
				GRAVITATIONAL_ACCELERATION_MS2 = .5, M_OVER_K = DEFAULT_MASS_KG / COEFFICIENT_OF_AIR_RESISTANCE_K,
				INSIDE_DIAMETER = Units.inchesToMeters(4), LENGTH = Units.inchesToMeters(12),
				OUTSIDE_DIAMETER = Units.inchesToMeters(4.5);
	}

}