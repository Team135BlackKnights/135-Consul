package frc.robot.commands.drive.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.GeometryConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.Vision.PreferredObjDetectObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.vision.VisionConstants;

import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.ObjDetectTxyObservation;

public class AimToObject extends Command {
  private static final double TRENCH_WALL_THICKNESS_METERS = Units.inchesToMeters(12.0);
  private static final double MAX_SPEED_METERS_PER_SEC = 4.5;
  private static final double MAX_ROTATION_RAD_PER_SEC = 15.0;
  private static final double MAX_ACCEL_METERS_PER_SEC_SQ = 4.0;
  private static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ = 8.0;
  private static final double SEARCH_CIRCLE_RADIUS_METERS = 2.5;
  private static final double SEARCH_POSE_TOLERANCE_METERS = 0.2;
  private static final double SEARCH_DRIVE_KP = 2.0;
  private static final double SEARCH_MAX_SPEED_METERS_PER_SEC = 2.0;
  private static final double SEARCH_ROTATION_KP = 4.0;
  private static final double SEARCH_OUT_OF_ZONE_SPIN_RAD_PER_SEC = Units.degreesToRadians(180.0);
  private static final double SEARCH_BIAS_DEG = 15.0;
  private static final double SEARCH_SWEEP_DEG = 20.0;
  private static final double SEARCH_SWEEP_PERIOD_SEC = 1.5;
  private static final double SEARCH_FIELD_EDGE_MARGIN_METERS = 0.35;
  private static final double AUTO_CENTER_LINE_MARGIN_METERS = 0.15;
  private static final double SEARCH_CIRCLE_DRIVER_ASSIST_DISTANCE_METERS = 0.25;
  private static final double TOWER_NO_GO_MARGIN_METERS = 0.3;
  private static final double TRENCH_WALL_NO_GO_MARGIN_METERS = 0.15;
  private static final double INTAKE_SAFETY_MARGIN_METERS = 0.0;
  private static final double TX_TOLERANCE_RAD = 0.05;
  private static final double DISTANCE_TOLERANCE_METERS = Units.inchesToMeters(.05);
  private static final double STALE_TIME_SEC = 0.5;
  private static final double VELOCITY_COMPARE_EPSILON = 1e-4;
  private static final double BLIND_INTAKE_COMMIT_SEC = 0.06;
  private static final double BLIND_INTAKE_COMMIT_MAX_TARGET_DISTANCE_METERS = 0.25;
  private static final double BLIND_INTAKE_COMMIT_MIN_FORWARD_SPEED_METERS_PER_SEC = .5;
  private static final double BLIND_INTAKE_COMMIT_MAX_FORWARD_SPEED_METERS_PER_SEC = 2.0;

  private final DrivetrainS drive;
  private final CameraID cam;
  private final int desiredClassId;
  private final double desiredDistanceMeters;

  private final LoggableTunedNumber kPTx = new LoggableTunedNumber("AimToObjectTx/kP", 4,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber kDTx = new LoggableTunedNumber("AimToObjectTx/kD", 0.6,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber kPDistance = new LoggableTunedNumber("AimToObjectTx/kPDistance", 3,
      TuningConstants.isTuningMacros);
  private final LoggableTunedNumber wallSlowDistanceMeters =
      new LoggableTunedNumber("AimToObjectTx/WallSlowDistanceMeters", 0.9, TuningConstants.isTuningMacros);
  private final LoggableTunedNumber wallMaxApproachSpeedMetersPerSec =
      new LoggableTunedNumber("AimToObjectTx/WallMaxApproachSpeedMetersPerSec", 1, TuningConstants.isTuningMacros);

  private double prevTxRadians = 0.0; // raw tx (negative=left)
  private double latestTxRadians = 0.0; // raw tx (negative=left)
  private double latestTyRadians = 0.0;
  private double latestDistanceMeters = 0.0;
  private int latestClusterCount = 0;
  private double latestClusterScore = 0.0;
  private boolean hasValidObservation = false;
  private boolean isFinished = false;
  private double searchStartTimestampSec = 0.0;
  private double lastValidObservationTimestampSec = Double.NEGATIVE_INFINITY;
  private Translation2d latestObjectFieldTarget = Translation2d.kZero;
  private boolean latestObservationBlockedByTower = false;
  private SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SEC_SQ);
  private SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SEC_SQ);
  private SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ);
  private ChassisVelocities lastLimitedSpeeds = new ChassisVelocities();
  private boolean slowingDueToWall = false;
  private boolean stoppedDueToTowerNoGo = false;
  private boolean stoppedDueToTrenchWallNoGo = false;
  private boolean blindIntakeCommitActive = false;
  private String motionLimitReason = "None";

  /**
   * Aim at the preferred detected object cluster in a given camera.
   *
   * @param drive                 drivetrain
   * @param cam                   which camera to use
   * @param desiredClassId        object class to aim at (-1 means any class)
   * @param desiredDistanceMeters standoff distance to maintain (0 means just aim
   *                              and GO TOWARDS until we can't see it anymore)
   */
  public AimToObject(DrivetrainS drive,
      CameraID cam,
      int desiredClassId,
      double desiredDistanceMeters) {
    this.drive = drive;
    this.cam = cam;
    this.desiredClassId = desiredClassId;
    this.desiredDistanceMeters = desiredDistanceMeters;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    RobotContainer.currentPath = "AIMTOOBJECT_CAM_" + cam + "_CLASS_" + desiredClassId;
    prevTxRadians = 0.0;
    isFinished = false;
    searchStartTimestampSec = Timer.getTimestamp();
    lastValidObservationTimestampSec = Double.NEGATIVE_INFINITY;
    configureAccelerationLimiters(drive.getChassisVelocities());
  }

  @Override
  public void execute() {
    hasValidObservation = false;
    latestObservationBlockedByTower = false;
    latestClusterCount = 0;
    latestClusterScore = 0.0;
    Logger.recordOutput("Drive/AimToObject/SearchEnabled", false);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistEnabled", false);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVx", 0.0);
    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVy", 0.0);
    Logger.recordOutput("Drive/AimToObject/SearchDistance", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusMeters", Double.NaN);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusErrorMeters", Double.NaN);
    resetMotionLimitDiagnostics();
    blindIntakeCommitActive = false;
    Pose2d robotPose = drive.getPose();
    Optional<PreferredObjDetectObservation> preferredObsOpt =
        RobotContainer.visionS.getPreferredObjDetectObservation(cam, desiredClassId);

    if (preferredObsOpt.isPresent()) {
      ObjDetectTxyObservation obs = preferredObsOpt.get().observation();
      double observationAgeSec = TimeUtil.getLogTimeSeconds() - obs.timestamp();
      boolean freshOk = observationAgeSec < STALE_TIME_SEC;
      if (freshOk) {
        latestTxRadians = obs.tx().getRadians();
        latestTyRadians = obs.ty().getRadians();
        latestDistanceMeters = obs.distanceMeters();
        latestClusterCount = preferredObsOpt.get().clusterCount();
        latestClusterScore = preferredObsOpt.get().clusterScore();

        Translation2d projectedObjectField =
            GeomUtil.projectObjectObservationToField(
                robotPose,
                getRobotToCameraPose(),
                Rotation2d.fromRadians(latestTxRadians),
                Rotation2d.fromRadians(latestTyRadians),
                latestDistanceMeters);
        Logger.recordOutput(
            "Drive/AimToObject/ObjectFieldProjected",
            new Pose2d(projectedObjectField, robotPose.getRotation()));

        if (isInsideTowerNoGoZone(projectedObjectField)) {
          latestObservationBlockedByTower = true;
        } else {
          hasValidObservation = true;
          lastValidObservationTimestampSec = TimeUtil.getLogTimeSeconds();
          latestObjectFieldTarget = limitAutoIntakeTargetToAllianceSide(projectedObjectField);
        }
      }
    }
    ChassisVelocities rawDesiredSpeeds = hasValidObservation
        ? buildObjectTrackingSpeeds(robotPose, latestObjectFieldTarget)
        : shouldBlindIntakeCommit(robotPose)
            ? buildBlindIntakeCommitSpeeds(robotPose)
            : buildSearchSpeeds(robotPose);
    ChassisVelocities desiredSpeeds = limitCommandAcceleration(rawDesiredSpeeds);

    drive.setChassisVelocities(desiredSpeeds);

    Logger.recordOutput("Drive/AimToObject/HasObservation", hasValidObservation);
    Logger.recordOutput("Drive/AimToObject/ObservationBlockedByTower", latestObservationBlockedByTower);
    Logger.recordOutput("Drive/AimToObject/txRad", latestTxRadians);
    Logger.recordOutput("Drive/AimToObject/tyRad", latestTyRadians);
    Logger.recordOutput("Drive/AimToObject/distance", latestDistanceMeters);
    Logger.recordOutput("Drive/AimToObject/ClusterCount", latestClusterCount);
    Logger.recordOutput("Drive/AimToObject/ClusterScore", latestClusterScore);
    Logger.recordOutput("Drive/AimToObject/RawCommandVx", rawDesiredSpeeds.vx);
    Logger.recordOutput("Drive/AimToObject/RawCommandVy", rawDesiredSpeeds.vy);
    Logger.recordOutput("Drive/AimToObject/RawCommandOmega", rawDesiredSpeeds.omega);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandVx", desiredSpeeds.vx);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandVy", desiredSpeeds.vy);
    Logger.recordOutput("Drive/AimToObject/LimitedCommandOmega", desiredSpeeds.omega);
    Logger.recordOutput("Drive/AimToObject/SlowingDueToWall", slowingDueToWall);
    Logger.recordOutput("Drive/AimToObject/StoppedDueToTowerNoGo", stoppedDueToTowerNoGo);
    Logger.recordOutput("Drive/AimToObject/StoppedDueToTrenchWallNoGo", stoppedDueToTrenchWallNoGo);
    Logger.recordOutput("Drive/AimToObject/BlindIntakeCommitActive", blindIntakeCommitActive);
    Logger.recordOutput("Drive/AimToObject/MotionLimitReason", motionLimitReason);
  }

  private void configureAccelerationLimiters(ChassisVelocities seedSpeeds) {
    xVelocityLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SEC_SQ);
    yVelocityLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SEC_SQ);
    angularVelocityLimiter = new SlewRateLimiter(MAX_ANGULAR_ACCEL_RAD_PER_SEC_SQ);
    xVelocityLimiter.reset(seedSpeeds.vx);
    yVelocityLimiter.reset(seedSpeeds.vy);
    angularVelocityLimiter.reset(seedSpeeds.omega);
    lastLimitedSpeeds = seedSpeeds;
  }

  private ChassisVelocities limitCommandAcceleration(ChassisVelocities desiredSpeeds) {
    lastLimitedSpeeds = new ChassisVelocities(
        xVelocityLimiter.calculate(desiredSpeeds.vx),
        yVelocityLimiter.calculate(desiredSpeeds.vy),
        angularVelocityLimiter.calculate(desiredSpeeds.omega));
    return lastLimitedSpeeds;
  }

  private ChassisVelocities buildObjectTrackingSpeeds(Pose2d robotPose, Translation2d objectField) {
    double angularCommand = buildAngularCommandFromLatestObservation();
    double forwardCommand = 0.0;

    double distanceError = latestDistanceMeters - desiredDistanceMeters;
    if (desiredDistanceMeters <= 1e-6 || Math.abs(distanceError) > DISTANCE_TOLERANCE_METERS) {
      forwardCommand = kPDistance.get() * distanceError;
      forwardCommand = frc.robot.utils.maths.CommonMath.clamp(forwardCommand, -MAX_SPEED_METERS_PER_SEC, MAX_SPEED_METERS_PER_SEC);
    }

    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    Translation2d captureIntakePoint = getCaptureIntakePoint(robotPose, objectField);
    Translation2d robotToObject = getCaptureErrorVector(robotPose, objectField);
    Translation2d driveVelocity = Translation2d.kZero;
    if (robotToObject.getNorm() > 1e-6 && Math.abs(forwardCommand) > 1e-6) {
      driveVelocity = robotToObject.div(robotToObject.getNorm()).times(forwardCommand);
      driveVelocity = applyMotionLimits(driveVelocity, protectedIntakePoint, objectField, robotPose, angularCommand);
    }

    prevTxRadians = latestTxRadians;
    Logger.recordOutput("Drive/AimToObject/ObjectFieldPose", new Pose2d(objectField, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/ProtectedIntakePoint", new Pose2d(protectedIntakePoint, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/CaptureIntakePoint", new Pose2d(captureIntakePoint, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/forwardCommand", forwardCommand);
    Logger.recordOutput("Drive/AimToObject/angularCommand", angularCommand);
    return frc.robot.utils.drive.ChassisVelocityUtil.fromFieldRelative(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private ChassisVelocities buildBlindIntakeCommitSpeeds(Pose2d robotPose) {
    blindIntakeCommitActive = true;

    double angularCommand = buildAngularCommandFromLatestObservation();
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    Translation2d captureIntakePoint = getCaptureIntakePoint(robotPose, latestObjectFieldTarget);
    double distanceToLastTarget = getCaptureErrorVector(robotPose, latestObjectFieldTarget).getNorm();
    double forwardSpeed = frc.robot.utils.maths.CommonMath.clamp(
        kPDistance.get() * distanceToLastTarget,
        BLIND_INTAKE_COMMIT_MIN_FORWARD_SPEED_METERS_PER_SEC,
        BLIND_INTAKE_COMMIT_MAX_FORWARD_SPEED_METERS_PER_SEC);

    Translation2d driveVelocity = new Translation2d(forwardSpeed, robotPose.getRotation());
    driveVelocity = applyMotionLimits(driveVelocity, protectedIntakePoint, latestObjectFieldTarget, robotPose, angularCommand);

    Logger.recordOutput("Drive/AimToObject/BlindIntakeCommitDistance", distanceToLastTarget);
    Logger.recordOutput(
        "Drive/AimToObject/BlindIntakeCommitAgeSec",
        TimeUtil.getLogTimeSeconds() - lastValidObservationTimestampSec);
    Logger.recordOutput("Drive/AimToObject/forwardCommand", forwardSpeed);
    Logger.recordOutput("Drive/AimToObject/angularCommand", angularCommand);
    Logger.recordOutput(
        "Drive/AimToObject/ProtectedIntakePoint",
        new Pose2d(protectedIntakePoint, robotPose.getRotation()));
    Logger.recordOutput(
        "Drive/AimToObject/CaptureIntakePoint",
        new Pose2d(captureIntakePoint, robotPose.getRotation()));
    return frc.robot.utils.drive.ChassisVelocityUtil.fromFieldRelative(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private double buildAngularCommandFromLatestObservation() {
    double dTx = latestTxRadians - prevTxRadians;
    if (Math.abs(latestTxRadians) <= TX_TOLERANCE_RAD) {
      return 0.0;
    }
    return frc.robot.utils.maths.CommonMath.clamp(
        -(kPTx.get() * latestTxRadians + kDTx.get() * dTx),
        -MAX_ROTATION_RAD_PER_SEC,
        MAX_ROTATION_RAD_PER_SEC);
  }

  private ChassisVelocities buildSearchSpeeds(Pose2d robotPose) {
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    Translation2d hubCenter = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();
    Translation2d fromHub = protectedIntakePoint.minus(hubCenter);
    if (fromHub.getNorm() < 1e-6) {
      fromHub = new Translation2d(1.0, 0.0);
    }
    double currentSearchRadiusMeters = fromHub.getNorm();
    double searchRadiusErrorMeters = Math.abs(currentSearchRadiusMeters - SEARCH_CIRCLE_RADIUS_METERS);

    if (!isInAllianceZone(robotPose)) {
      double angularCommand = frc.robot.utils.maths.CommonMath.clamp(
          Robot.isRed ? -SEARCH_OUT_OF_ZONE_SPIN_RAD_PER_SEC : SEARCH_OUT_OF_ZONE_SPIN_RAD_PER_SEC,
          -MAX_ROTATION_RAD_PER_SEC,
          MAX_ROTATION_RAD_PER_SEC);

      Logger.recordOutput("Drive/AimToObject/SearchEnabled", true);
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistEnabled", false);
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVx", 0.0);
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVy", 0.0);
      Logger.recordOutput(
          "Drive/AimToObject/SearchPose",
          new Pose2d(robotPose.getTranslation(), robotPose.getRotation()));
      Logger.recordOutput("Drive/AimToObject/SearchDistance", 0.0);
      Logger.recordOutput("Drive/AimToObject/SearchRadiusMeters", currentSearchRadiusMeters);
      Logger.recordOutput("Drive/AimToObject/SearchRadiusErrorMeters", searchRadiusErrorMeters);
      Logger.recordOutput(
          "Drive/AimToObject/ProtectedIntakePoint",
          new Pose2d(protectedIntakePoint, robotPose.getRotation()));
      Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", angularCommand);
      return new ChassisVelocities(0.0, 0.0, angularCommand);
    }

    Logger.recordOutput("Drive/AimToObject/SearchEnabled", true);
    Pose2d searchPose = getSearchPose(robotPose);
    Translation2d toSearchPose = searchPose.getTranslation().minus(protectedIntakePoint);
    double distanceToSearchPose = toSearchPose.getNorm();

    Translation2d driveVelocity = Translation2d.kZero;
    if (distanceToSearchPose > SEARCH_POSE_TOLERANCE_METERS) {
      double speed = Math.min(SEARCH_MAX_SPEED_METERS_PER_SEC, SEARCH_DRIVE_KP * distanceToSearchPose);
      driveVelocity = toSearchPose.div(distanceToSearchPose).times(speed);
      driveVelocity = applyMotionLimits(driveVelocity, protectedIntakePoint, searchPose.getTranslation(), robotPose, 0.0);
    }

    boolean searchCircleDriverAssistEnabled =
        RobotState.isTeleopEnabled()
            && fromHub.getNorm() > 1e-6
            && searchRadiusErrorMeters <= SEARCH_CIRCLE_DRIVER_ASSIST_DISTANCE_METERS;
    if (searchCircleDriverAssistEnabled) {
      Translation2d tangentialDriverVelocity = getSearchCircleTangentialDriverVelocity(fromHub);
      driveVelocity = driveVelocity.plus(tangentialDriverVelocity);
      if (driveVelocity.getNorm() > SEARCH_MAX_SPEED_METERS_PER_SEC) {
        driveVelocity = driveVelocity.div(driveVelocity.getNorm()).times(SEARCH_MAX_SPEED_METERS_PER_SEC);
      }
      driveVelocity = applyMotionLimits(driveVelocity, protectedIntakePoint, searchPose.getTranslation(), robotPose, 0.0);
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVx", tangentialDriverVelocity.getX());
      Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistVy", tangentialDriverVelocity.getY());
    }

    double headingErrorRad = searchPose.getRotation().minus(robotPose.getRotation()).getRadians();
    double angularCommand = frc.robot.utils.maths.CommonMath.clamp(
        SEARCH_ROTATION_KP * headingErrorRad,
        -MAX_ROTATION_RAD_PER_SEC,
        MAX_ROTATION_RAD_PER_SEC);

    Logger.recordOutput("Drive/AimToObject/SearchCircleDriverAssistEnabled", searchCircleDriverAssistEnabled);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusMeters", currentSearchRadiusMeters);
    Logger.recordOutput("Drive/AimToObject/SearchRadiusErrorMeters", searchRadiusErrorMeters);
    Logger.recordOutput("Drive/AimToObject/SearchPose", searchPose);
    Logger.recordOutput("Drive/AimToObject/SearchDistance", distanceToSearchPose);
    Logger.recordOutput("Drive/AimToObject/ProtectedIntakePoint", new Pose2d(protectedIntakePoint, robotPose.getRotation()));
    Logger.recordOutput("Drive/AimToObject/SearchAngularCommand", angularCommand);
    return frc.robot.utils.drive.ChassisVelocityUtil.fromFieldRelative(
        driveVelocity.getX(),
        driveVelocity.getY(),
        angularCommand,
        robotPose.getRotation());
  }

  private Translation2d getSearchCircleTangentialDriverVelocity(Translation2d fromHub) {
    Translation2d rawDriverVelocity = getDriverFieldVelocity();
    Translation2d tangentUnit = fromHub.div(fromHub.getNorm()).rotateBy(Rotation2d.fromDegrees(90.0));
    double tangentialSpeedMetersPerSec =
        rawDriverVelocity.getX() * tangentUnit.getX() + rawDriverVelocity.getY() * tangentUnit.getY();
    return tangentUnit.times(tangentialSpeedMetersPerSec);
  }

  private Translation2d getDriverFieldVelocity() {
    double xSpeed = -MathUtil.applyDeadband(
        RobotContainer.driveController.getLeftY(),
        DriveConstants.DriverConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = -MathUtil.applyDeadband(
        RobotContainer.driveController.getLeftX(),
        DriveConstants.DriverConstants.kDeadband) * DriveConstants.kMaxSpeedMetersPerSecond;
    if (Robot.isRed) {
      xSpeed *= -1.0;
      ySpeed *= -1.0;
    }
    return new Translation2d(xSpeed, ySpeed);
  }

  private Pose2d getSearchPose(Pose2d robotPose) {
    Translation2d hubCenter = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();
    Translation2d fromHub = getProtectedIntakePoint(robotPose).minus(hubCenter);
    if (fromHub.getNorm() < 1e-6) {
      fromHub = new Translation2d(1.0, 0.0);
    }

    Translation2d searchPoint =
        hubCenter.plus(fromHub.div(fromHub.getNorm()).times(SEARCH_CIRCLE_RADIUS_METERS));
    searchPoint = clampToField(searchPoint, SEARCH_FIELD_EDGE_MARGIN_METERS);
    searchPoint = pushSearchPointOutOfTowerNoGoZone(searchPoint);
    searchPoint = pushSearchPointOutOfTrenchWallNoGoZone(searchPoint);
    searchPoint = limitAutoIntakeTargetToAllianceSide(searchPoint);
    searchPoint = pushSearchPointOutOfTowerNoGoZone(searchPoint);
    searchPoint = pushSearchPointOutOfTrenchWallNoGoZone(searchPoint);
    return new Pose2d(searchPoint, getSearchHeading(searchPoint, hubCenter));
  }

  private Translation2d getProtectedIntakePoint(Pose2d robotPose) {
    return robotPose.getTranslation()
        .plus(new Translation2d(getProtectedIntakeReachMeters(), robotPose.getRotation()));
  }

  private Translation2d getProtectedIntakePoint(Translation2d robotPosition, Rotation2d robotHeading) {
    return robotPosition.plus(new Translation2d(getProtectedIntakeReachMeters(), robotHeading));
  }

  private Translation2d getCaptureIntakePoint(Pose2d robotPose, Translation2d objectField) {
    Translation2d relativeObject = objectField
        .minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().unaryMinus());
    Translation2d capturePointRobotRelative = new Translation2d(
        frc.robot.utils.maths.CommonMath.clamp(relativeObject.getX(), getCaptureZoneMinX(), getCaptureZoneMaxX()),
        frc.robot.utils.maths.CommonMath.clamp(relativeObject.getY(), -getCaptureZoneHalfWidth(), getCaptureZoneHalfWidth()));
    return robotPose.getTranslation().plus(capturePointRobotRelative.rotateBy(robotPose.getRotation()));
  }

  private Translation2d getCaptureErrorVector(Pose2d robotPose, Translation2d objectField) {
    return objectField.minus(getCaptureIntakePoint(robotPose, objectField));
  }

  private double getProtectedIntakeReachMeters() {
    return Math.max(0.0, GeometryConstants.simIntakeFrontEdgeFromRobotCenter)
        + Math.max(0.0, GeometryConstants.simIntakeDepth)
        + Math.max(0.0, INTAKE_SAFETY_MARGIN_METERS);
  }

  private double getCaptureZoneMinX() {
    return GeometryConstants.simIntakeFrontEdgeFromRobotCenter - FieldConstants.FUEL_DIAMETER / 2.0;
  }

  private double getCaptureZoneMaxX() {
    return GeometryConstants.simIntakeFrontEdgeFromRobotCenter
        + GeometryConstants.simIntakeDepth
        + FieldConstants.FUEL_DIAMETER / 2.0;
  }

  private double getCaptureZoneHalfWidth() {
    return GeometryConstants.simIntakeWidth / 2.0 + FieldConstants.FUEL_DIAMETER / 2.0;
  }

  private boolean isInAllianceZone(Pose2d robotPose) {
    if (Robot.isRed) {
      return robotPose.getX() >= FieldConstants.LinesVertical.oppAllianceZone;
    }
    return robotPose.getX() <= FieldConstants.LinesVertical.allianceZone;
  }

  private Rotation2d getSearchHeading(Translation2d searchPoint, Translation2d hubCenter) {
    Rotation2d outwardHeading = searchPoint.minus(hubCenter).getAngle();
    double biasSign = searchPoint.getY() >= FieldConstants.FIELD_HEIGHT / 2.0 ? -1.0 : 1.0;
    Rotation2d biasedHeading = outwardHeading.rotateBy(Rotation2d.fromDegrees(biasSign * SEARCH_BIAS_DEG));
    double sweepPeriod = Math.max(SEARCH_SWEEP_PERIOD_SEC, 0.1);
    double sweepRadians = Units.degreesToRadians(SEARCH_SWEEP_DEG)
        * Math.sin((Timer.getTimestamp() - searchStartTimestampSec) * 2.0 * Math.PI / sweepPeriod);
    return biasedHeading.rotateBy(Rotation2d.fromRadians(sweepRadians));
  }

  private Pose3d getRobotToCameraPose() {
    return VisionConstants.cameras[cam.ordinal()].getPose().get();
  }

  private Translation2d clampToField(Translation2d point, double margin) {
    return new Translation2d(
        frc.robot.utils.maths.CommonMath.clamp(point.getX(), margin, FieldConstants.FIELD_WIDTH - margin),
        frc.robot.utils.maths.CommonMath.clamp(point.getY(), margin, FieldConstants.FIELD_HEIGHT - margin));
  }

  private Translation2d limitAutoIntakeTargetToAllianceSide(Translation2d point) {
    if (!RobotState.isAutonomousEnabled()) {
      return point;
    }

    double centerLineX = FieldConstants.LinesVertical.center;
    double margin = Math.max(0.0, AUTO_CENTER_LINE_MARGIN_METERS);
    if (Robot.isRed) {
      return new Translation2d(Math.max(point.getX(), centerLineX + margin), point.getY());
    }
    return new Translation2d(Math.min(point.getX(), centerLineX - margin), point.getY());
  }

  private Translation2d limitAutoIntakeVelocityToAllianceSide(
      Translation2d desiredFieldVelocity,
      Translation2d robotPosition) {
    if (!RobotState.isAutonomousEnabled()) {
      return desiredFieldVelocity;
    }

    double centerLineX = FieldConstants.LinesVertical.center;
    double margin = Math.max(0.0, AUTO_CENTER_LINE_MARGIN_METERS);
    double dtSec = 0.02;
    double limitedX = desiredFieldVelocity.getX();

    if (Robot.isRed) {
      double minAllowedX = centerLineX + margin;
      double maxNegativeSpeed = (robotPosition.getX() - minAllowedX) / dtSec;
      if (limitedX < 0.0) {
        limitedX = Math.max(limitedX, -Math.max(0.0, maxNegativeSpeed));
      }
    } else {
      double maxAllowedX = centerLineX - margin;
      double maxPositiveSpeed = (maxAllowedX - robotPosition.getX()) / dtSec;
      if (limitedX > 0.0) {
        limitedX = Math.min(limitedX, Math.max(0.0, maxPositiveSpeed));
      }
    }

    return new Translation2d(limitedX, desiredFieldVelocity.getY());
  }

  private Translation2d pushSearchPointOutOfTowerNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, TOWER_NO_GO_MARGIN_METERS);

    double blueMinY = Math.min(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) - margin;
    double blueMaxY = Math.max(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) + margin;
    double blueMaxX = FieldConstants.Tower.frontFaceX + margin;
    if (point.getX() <= blueMaxX && point.getY() >= blueMinY && point.getY() <= blueMaxY) {
      return new Translation2d(blueMaxX + 1e-3, point.getY());
    }

    double redMinY =
        Math.min(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) - margin;
    double redMaxY =
        Math.max(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) + margin;
    double redMinX = FieldConstants.FIELD_WIDTH - FieldConstants.Tower.frontFaceX - margin;
    if (point.getX() >= redMinX && point.getY() >= redMinY && point.getY() <= redMaxY) {
      return new Translation2d(redMinX - 1e-3, point.getY());
    }

    return point;
  }

  private Translation2d pushSearchPointOutOfTrenchWallNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, TRENCH_WALL_NO_GO_MARGIN_METERS);
    double blueMinX = getBlueAllianceTrenchWallMinX(margin);
    double blueMaxX = getBlueAllianceTrenchWallMaxX(margin);
    double redMinX = getRedAllianceTrenchWallMinX(margin);
    double redMaxX = getRedAllianceTrenchWallMaxX(margin);
    double bottomMinY = getBottomTrenchWallMinY(margin);
    double bottomMaxY = getBottomTrenchWallMaxY(margin);
    double topMinY = getTopTrenchWallMinY(margin);
    double topMaxY = getTopTrenchWallMaxY(margin);

    if (point.getX() >= blueMinX && point.getX() <= blueMaxX
        && point.getY() >= bottomMinY && point.getY() <= bottomMaxY) {
      return new Translation2d(point.getX(), bottomMaxY + 1e-3);
    }

    if (point.getX() >= blueMinX && point.getX() <= blueMaxX
        && point.getY() >= topMinY && point.getY() <= topMaxY) {
      return new Translation2d(point.getX(), topMinY - 1e-3);
    }

    if (point.getX() >= redMinX && point.getX() <= redMaxX
        && point.getY() >= bottomMinY && point.getY() <= bottomMaxY) {
      return new Translation2d(point.getX(), bottomMaxY + 1e-3);
    }

    if (point.getX() >= redMinX && point.getX() <= redMaxX
        && point.getY() >= topMinY && point.getY() <= topMaxY) {
      return new Translation2d(point.getX(), topMinY - 1e-3);
    }

    return point;
  }

  private boolean isInsideTowerNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, TOWER_NO_GO_MARGIN_METERS);

    double blueMinY = Math.min(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) - margin;
    double blueMaxY = Math.max(FieldConstants.Tower.leftUpright.getY(), FieldConstants.Tower.rightUpright.getY()) + margin;
    boolean insideBlueTower =
        point.getX() >= 0.0
            && point.getX() <= FieldConstants.Tower.frontFaceX + margin
            && point.getY() >= blueMinY
            && point.getY() <= blueMaxY;
    if (insideBlueTower) {
      return true;
    }

    double redMinY =
        Math.min(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) - margin;
    double redMaxY =
        Math.max(FieldConstants.Tower.oppLeftUpright.getY(), FieldConstants.Tower.oppRightUpright.getY()) + margin;
    return point.getX() >= FieldConstants.FIELD_WIDTH - FieldConstants.Tower.frontFaceX - margin
        && point.getX() <= FieldConstants.FIELD_WIDTH
        && point.getY() >= redMinY
        && point.getY() <= redMaxY;
  }

  private boolean isInsideTrenchWallNoGoZone(Translation2d point) {
    double margin = Math.max(0.0, TRENCH_WALL_NO_GO_MARGIN_METERS);

    double blueMinX = getBlueAllianceTrenchWallMinX(margin);
    double blueMaxX = getBlueAllianceTrenchWallMaxX(margin);
    double redMinX = getRedAllianceTrenchWallMinX(margin);
    double redMaxX = getRedAllianceTrenchWallMaxX(margin);
    double bottomMinY = getBottomTrenchWallMinY(margin);
    double bottomMaxY = getBottomTrenchWallMaxY(margin);
    double topMinY = getTopTrenchWallMinY(margin);
    double topMaxY = getTopTrenchWallMaxY(margin);

    boolean insideBlueBottom =
        point.getX() >= blueMinX
            && point.getX() <= blueMaxX
            && point.getY() >= bottomMinY
            && point.getY() <= bottomMaxY;
    if (insideBlueBottom) {
      return true;
    }

    boolean insideBlueTop =
        point.getX() >= blueMinX
            && point.getX() <= blueMaxX
            && point.getY() >= topMinY
            && point.getY() <= topMaxY;
    if (insideBlueTop) {
      return true;
    }

    boolean insideRedBottom =
        point.getX() >= redMinX
            && point.getX() <= redMaxX
            && point.getY() >= bottomMinY
            && point.getY() <= bottomMaxY;
    if (insideRedBottom) {
      return true;
    }

    return point.getX() >= redMinX
        && point.getX() <= redMaxX
        && point.getY() >= topMinY
        && point.getY() <= topMaxY;
  }

  private double getBlueAllianceTrenchWallMinX(double margin) {
    return FieldConstants.Hub.nearRightCorner.getX() - margin;
  }

  private double getBlueAllianceTrenchWallMaxX(double margin) {
    return FieldConstants.Hub.nearRightCorner.getX() + FieldConstants.RightTrench.depth + margin;
  }

  private double getRedAllianceTrenchWallMinX(double margin) {
    return FieldConstants.Hub.oppNearRightCorner.getX() - margin;
  }

  private double getRedAllianceTrenchWallMaxX(double margin) {
    return FieldConstants.Hub.oppNearRightCorner.getX() + FieldConstants.RightTrench.depth + margin;
  }

  private double getBottomTrenchWallMinY(double margin) {
    return FieldConstants.LinesHorizontal.rightTrenchOpenStart - margin;
  }

  private double getBottomTrenchWallMaxY(double margin) {
    return FieldConstants.LinesHorizontal.rightBumpEnd + margin;
  }

  private double getTopTrenchWallMinY(double margin) {
    return FieldConstants.LinesHorizontal.leftBumpStart - margin;
  }

  private double getTopTrenchWallMaxY(double margin) {
    return FieldConstants.LinesHorizontal.leftTrenchOpenEnd + margin;
  }

  private Translation2d limitVelocityToAvoidTowerNoGoZone(
      Translation2d desiredFieldVelocity,
      Pose2d robotPose,
      double desiredAngularVelocityRadPerSec) {
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    if (desiredFieldVelocity.getNorm() <= 1e-6 || isInsideTowerNoGoZone(protectedIntakePoint)) {
      return desiredFieldVelocity;
    }

    Translation2d nextRobotPosition = robotPose.getTranslation().plus(desiredFieldVelocity.times(0.02));
    Rotation2d nextRobotHeading =
        robotPose.getRotation().plus(Rotation2d.fromRadians(desiredAngularVelocityRadPerSec * 0.02));
    Translation2d nextProtectedIntakePoint = getProtectedIntakePoint(nextRobotPosition, nextRobotHeading);
    if (isInsideTowerNoGoZone(nextProtectedIntakePoint)) {
      return Translation2d.kZero;
    }

    return desiredFieldVelocity;
  }

  private Translation2d limitVelocityToAvoidTrenchWallNoGoZone(
      Translation2d desiredFieldVelocity,
      Pose2d robotPose,
      double desiredAngularVelocityRadPerSec) {
    Translation2d protectedIntakePoint = getProtectedIntakePoint(robotPose);
    if (desiredFieldVelocity.getNorm() <= 1e-6 || isInsideTrenchWallNoGoZone(protectedIntakePoint)) {
      return desiredFieldVelocity;
    }

    Translation2d nextRobotPosition = robotPose.getTranslation().plus(desiredFieldVelocity.times(0.02));
    Rotation2d nextRobotHeading =
        robotPose.getRotation().plus(Rotation2d.fromRadians(desiredAngularVelocityRadPerSec * 0.02));
    Translation2d nextProtectedIntakePoint = getProtectedIntakePoint(nextRobotPosition, nextRobotHeading);
    if (isInsideTrenchWallNoGoZone(nextProtectedIntakePoint)) {
      return Translation2d.kZero;
    }

    return desiredFieldVelocity;
  }

  private void resetMotionLimitDiagnostics() {
    slowingDueToWall = false;
    stoppedDueToTowerNoGo = false;
    stoppedDueToTrenchWallNoGo = false;
    motionLimitReason = "None";
  }

  private boolean shouldBlindIntakeCommit(Pose2d robotPose) {
    if (desiredDistanceMeters > 1e-6) {
      return false;
    }

    double observationAgeSec = TimeUtil.getLogTimeSeconds() - lastValidObservationTimestampSec;
    if (observationAgeSec > BLIND_INTAKE_COMMIT_SEC) {
      return false;
    }

    return getCaptureErrorVector(robotPose, latestObjectFieldTarget)
        .getNorm() <= BLIND_INTAKE_COMMIT_MAX_TARGET_DISTANCE_METERS;
  }

  private Translation2d applyMotionLimits(
      Translation2d desiredFieldVelocity,
      Translation2d protectedIntakePoint,
      Translation2d targetPosition,
      Pose2d robotPose,
      double desiredAngularVelocityRadPerSec) {
    Translation2d limitedVelocity = GeomUtil.limitVelocityTowardFieldEdge(
        desiredFieldVelocity,
        protectedIntakePoint,
        targetPosition,
        wallSlowDistanceMeters.get(),
        wallMaxApproachSpeedMetersPerSec.get());
    if (velocityChanged(desiredFieldVelocity, limitedVelocity)) {
      slowingDueToWall = true;
      motionLimitReason = "Wall";
    }

    limitedVelocity = limitAutoIntakeVelocityToAllianceSide(limitedVelocity, protectedIntakePoint);

    Translation2d towerLimitedVelocity =
        limitVelocityToAvoidTowerNoGoZone(limitedVelocity, robotPose, desiredAngularVelocityRadPerSec);
    if (velocityChanged(limitedVelocity, towerLimitedVelocity)) {
      stoppedDueToTowerNoGo = true;
      motionLimitReason = "Tower";
    }
    limitedVelocity = towerLimitedVelocity;

    Translation2d trenchLimitedVelocity =
        limitVelocityToAvoidTrenchWallNoGoZone(limitedVelocity, robotPose, desiredAngularVelocityRadPerSec);
    if (velocityChanged(limitedVelocity, trenchLimitedVelocity)) {
      stoppedDueToTrenchWallNoGo = true;
      motionLimitReason = "Trench";
    }
    return trenchLimitedVelocity;
  }

  private boolean velocityChanged(Translation2d first, Translation2d second) {
    return first.minus(second).getNorm() > VELOCITY_COMPARE_EPSILON;
  }

  @Override
  public void end(boolean interrupted) {
    configureAccelerationLimiters(new ChassisVelocities());
    drive.setChassisVelocities(new ChassisVelocities(0, 0, 0));
    RobotContainer.currentPath = "";
    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
