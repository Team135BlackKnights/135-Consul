package frc.robot.utils.CompetitionFieldUtils.ReferencePoses;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;

public final class FieldReferencePoses {
  private static final Rotation2d ZERO_ROTATION = new Rotation2d();
  public static final double TRENCH_WALL_THICKNESS_METERS = Units.inchesToMeters(12.0);

  public static final Pose2d BLUE_HUB_CENTER = pose(FieldConstants.Hub.topCenterPoint);
  public static final Pose2d RED_HUB_CENTER = pose(FieldConstants.Hub.oppTopCenterPoint);
  public static final Pose2d[] BLUE_HUB_CORNERS = rectangle(
      FieldConstants.Hub.nearRightCorner.getX(),
      FieldConstants.Hub.nearRightCorner.getY(),
      FieldConstants.Hub.farLeftCorner.getX(),
      FieldConstants.Hub.farLeftCorner.getY());
  public static final Pose2d[] RED_HUB_CORNERS = rectangle(
      FieldConstants.Hub.oppNearRightCorner.getX(),
      FieldConstants.Hub.oppNearRightCorner.getY(),
      FieldConstants.Hub.oppFarLeftCorner.getX(),
      FieldConstants.Hub.oppFarLeftCorner.getY());

  public static final Pose2d BLUE_TOWER_CENTER = pose(FieldConstants.Tower.centerPoint);
  public static final Pose2d RED_TOWER_CENTER = pose(FieldConstants.Tower.oppCenterPoint);
  public static final Pose2d[] BLUE_TOWER_UPRIGHTS = new Pose2d[] {
      pose(FieldConstants.Tower.leftUpright),
      pose(FieldConstants.Tower.rightUpright)
  };
  public static final Pose2d[] RED_TOWER_UPRIGHTS = new Pose2d[] {
      pose(FieldConstants.Tower.oppLeftUpright),
      pose(FieldConstants.Tower.oppRightUpright)
  };
  public static final Pose2d[] BLUE_TOWER_NO_GO_CORNERS = rectangle(
      0.0,
      Math.min(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()),
      FieldConstants.Tower.frontFaceX,
      Math.max(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()));
  public static final Pose2d[] RED_TOWER_NO_GO_CORNERS = rectangle(
      FieldConstants.FIELD_WIDTH - FieldConstants.Tower.frontFaceX,
      Math.min(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()),
      FieldConstants.FIELD_WIDTH,
      Math.max(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()));

  public static final Pose2d BLUE_BOTTOM_TRENCH_OPENING_CENTER = pose(FieldConstants.RightTrench.openingCenter);
  public static final Pose2d BLUE_TOP_TRENCH_OPENING_CENTER = pose(FieldConstants.LeftTrench.openingCenter);
  public static final Pose2d RED_BOTTOM_TRENCH_OPENING_CENTER = pose(
      new Translation2d(
          FieldConstants.LinesVertical.oppHubCenter,
          FieldConstants.RightTrench.openingWidth / 2.0));
  public static final Pose2d RED_TOP_TRENCH_OPENING_CENTER = pose(
      new Translation2d(
          FieldConstants.LinesVertical.oppHubCenter,
          FieldConstants.FIELD_HEIGHT - (FieldConstants.LeftTrench.openingWidth / 2.0)));

  public static final Pose2d[] BLUE_BOTTOM_TRENCH_WALL_CORNERS = rectangle(
      getBlueAllianceTrenchWallMinX(),
      FieldConstants.LinesHorizontal.rightTrenchOpenStart,
      getBlueAllianceTrenchWallMaxX(),
      FieldConstants.LinesHorizontal.rightBumpEnd);
  public static final Pose2d[] BLUE_TOP_TRENCH_WALL_CORNERS = rectangle(
      getBlueAllianceTrenchWallMinX(),
      FieldConstants.LinesHorizontal.leftBumpStart,
      getBlueAllianceTrenchWallMaxX(),
      FieldConstants.LinesHorizontal.leftTrenchOpenEnd);
  public static final Pose2d[] RED_BOTTOM_TRENCH_WALL_CORNERS = rectangle(
      getRedAllianceTrenchWallMinX(),
      FieldConstants.LinesHorizontal.rightTrenchOpenStart,
      getRedAllianceTrenchWallMaxX(),
      FieldConstants.LinesHorizontal.rightBumpEnd);
  public static final Pose2d[] RED_TOP_TRENCH_WALL_CORNERS = rectangle(
      getRedAllianceTrenchWallMinX(),
      FieldConstants.LinesHorizontal.leftBumpStart,
      getRedAllianceTrenchWallMaxX(),
      FieldConstants.LinesHorizontal.leftTrenchOpenEnd);

  private FieldReferencePoses() {
  }

  private static Pose2d pose(Translation2d translation) {
    return new Pose2d(translation.getX(), translation.getY(), ZERO_ROTATION);
  }

  private static Pose2d pose(Translation3d translation) {
    return new Pose2d(translation.getX(), translation.getY(), ZERO_ROTATION);
  }

  private static double getBlueAllianceTrenchWallMinX() {
    return FieldConstants.Hub.nearRightCorner.getX();
  }

  private static double getBlueAllianceTrenchWallMaxX() {
    return getBlueAllianceTrenchWallMinX() + FieldConstants.RightTrench.depth;
  }

  private static double getRedAllianceTrenchWallMinX() {
    return FieldConstants.Hub.oppNearRightCorner.getX();
  }

  private static double getRedAllianceTrenchWallMaxX() {
    return getRedAllianceTrenchWallMinX() + FieldConstants.RightTrench.depth;
  }

  private static Pose2d[] rectangle(double minX, double minY, double maxX, double maxY) {
    return new Pose2d[] {
        new Pose2d(minX, minY, ZERO_ROTATION),
        new Pose2d(maxX, minY, ZERO_ROTATION),
        new Pose2d(maxX, maxY, ZERO_ROTATION),
        new Pose2d(minX, maxY, ZERO_ROTATION)
    };
  }
}
