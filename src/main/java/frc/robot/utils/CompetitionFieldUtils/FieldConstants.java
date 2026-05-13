package frc.robot.utils.CompetitionFieldUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.AprilTagLayoutType;


/**
 * Constants for the field MUST CREATE A NEW CLASS FOR EACH NEW GAME, @see
 * Reefscape2025FieldSimulation & @see Reefscape2025FieldObjects for example
 */
public class FieldConstants {
	public static final double FIELD_WIDTH = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength(); //was fieldLength
  	public static final double FIELD_HEIGHT = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth(); //was fieldWidth
	public static final double COEFFICIENT_OF_GRAVITY = 9.81;
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
		FUEL
	}
	 public static class LinesVertical {
    public static final double center = FIELD_WIDTH / 2.0;
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double center = FIELD_HEIGHT / 2.0;

    // Right of hub
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left of hub
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = FIELD_HEIGHT;
  }

  /** Hub related constants */
  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            FIELD_HEIGHT / 2.0,
            height);
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            FIELD_HEIGHT / 2.0,
            innerHeight);

    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, FIELD_HEIGHT / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, FIELD_HEIGHT	 / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, FIELD_HEIGHT / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, FIELD_HEIGHT / 2.0 - width / 2.0);

    // Relevant reference points on the opposite side
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            FIELD_HEIGHT / 2.0,
            height);
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, FIELD_HEIGHT / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, FIELD_HEIGHT / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, FIELD_HEIGHT / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, FIELD_HEIGHT / 2.0 - width / 2.0);

    // Hub faces
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
  }

  /** Left Bump related constants */
  public static class LeftBump {

    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right Bump related constants */
  public static class RightBump {
    // Dimensions
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // Relevant reference points on alliance side
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // Relevant reference points on opposing side
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left Trench related constants */
  public static class LeftTrench {
    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, FIELD_HEIGHT, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, FIELD_HEIGHT - openingWidth, openingHeight);
    public static final Translation2d openingCenter =
        new Translation2d(LinesVertical.hubCenter, FIELD_HEIGHT - (openingWidth / 2.0));
    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, FIELD_HEIGHT, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, FIELD_HEIGHT - openingWidth, openingHeight);
  }

  public static class RightTrench {

    // Dimensions
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // Relevant reference points on alliance side
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);
    public static final Translation2d openingCenter =
        new Translation2d(LinesVertical.hubCenter, openingWidth / 2.0);

    // Relevant reference points on opposing side
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower related constants */
  public static class Tower {
    // Dimensions
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);

    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // Rung heights from the floor
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // Relevant reference points on opposing side
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            FIELD_WIDTH - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            FIELD_WIDTH - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            FIELD_WIDTH - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  public static class Depot {
    // Dimensions
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // Relevant reference points on alliance side
    public static final Translation3d depotCenter =
        new Translation3d(depth, (FIELD_HEIGHT / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (FIELD_HEIGHT / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (FIELD_HEIGHT / 2) + distanceFromCenterY - (width / 2), height);
  }

  public static class Outpost {
    // Dimensions
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

	/* Game Piece 1 Andymark Location */
	public static final double FUEL_HEIGHT = Units.inchesToMeters(2.955); // meters
	public static final double FUEL_DIAMETER = Units.inchesToMeters(5.91); // meters

	public enum GamePieceTag {
		ON_GROUND_FUEL_BALL, IN_ROBOT_FUEL_BALL, IN_AIR_FUEL_BALL
	}

	public static final class Fuel {
		public static final double DEFAULT_MASS_KG = Units.lbsToKilograms(0.5), LINEAR_DAMPING = 1.8,
				ANGULAR_DAMPING = 5, EDGE_COEFFICIENT_OF_FRICTION = 0.8,
				EDGE_COEFFICIENT_OF_RESTITUTION = 0.8, FUEL_COEFFICIENT_OF_AIR_RESISTANCE_K = 1.89, 
				GRAVITATIONAL_ACCELERATION_MS2 = .5, M_OVER_K = DEFAULT_MASS_KG / FUEL_COEFFICIENT_OF_AIR_RESISTANCE_K;
	}
}