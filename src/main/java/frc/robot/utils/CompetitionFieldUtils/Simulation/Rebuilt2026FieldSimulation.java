package frc.robot.utils.CompetitionFieldUtils.Simulation;

import static org.wpilib.units.Units.*;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.driverstation.*;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Rebuilt2026FieldObjects.FuelOnFieldSimulated;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Rebuilt2026FieldObjects.RebuiltFuelOnFly;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RebuiltHub;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.RebuiltOutpost;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import org.littletonrobotics.junction.Logger;

//YEARLYUPDATE: change this field (field bounds/obstacles and scoring locations) to match the year's competition
/**
 * field simulation for 2025 competition
 * 
 */
@SuppressWarnings("unused") // There are some unused methods/variables that are still useful to keep around for testing and tuning purposes
public class Rebuilt2026FieldSimulation extends CompetitionFieldSimulation {
	private static final double FIELD_X_MIN = 0.00000;
	private static final double FIELD_X_MAX = 16.54105;
	private static final double FIELD_Y_MIN = 0.00000;
	private static final double FIELD_Y_MAX = 8.06926;

	private static final double HUB_X_LEN = 1.19380;
	private static final double HUB_Y_LEN = 1.19380;
	private static final double HUB_X = 4.625594;
	private static final double HUB_Y = 4.03463;
	private static final double HUB_RAMP_LENGTH = Inches.of(73.0).in(Meters);

	private static final double UPRIGHT_X_LEN = Inches.of(3.5).in(Meters);
	private static final double UPRIGHT_Y_LEN = Inches.of(1.5).in(Meters);
	private static final double UPRIGHT_OFFSET_FROM_END_WALL = 1.06204;
	private static final double UPRIGHT_OFFSET_FROM_SIDE_WALL = 3.31524;
	private static final double UPRIGHT_Y_SPACING = Inches.of(33.75).in(Meters);

	private static final double TRENCH_WALL_Y_LEN = Inches.of(12.0).in(Meters);
	private static final double TRENCH_WALL_X_LEN = Inches.of(47.0).in(Meters);
	private static final double TRENCH_WALL_OFFSET_FROM_END_WALL = 4.61769;
	private static final double TRENCH_WALL_OFFSET_FROM_SIDE_WALL = 1.43113;
	// Raw-TOF fit from REAL DATA: v_ball ~= 0.001682 * flywheelRPM (m/s).
	private static final LoggableTunedNumber simBallSpeedMetersPerSecPerRPM =
			new LoggableTunedNumber("SimShot/BallSpeedMpsPerRPM", 0.001682, true);
	private static final LoggableTunedNumber simShotCooldownSec =
			new LoggableTunedNumber("SimShot/CooldownSec", 3, true);
	private static final LoggableTunedNumber simHoodToPitchScale =
			new LoggableTunedNumber("SimShot/HoodToPitchScale", 1.0, true);
	private static final LoggableTunedNumber simHoodToPitchOffsetDeg =
			new LoggableTunedNumber("SimShot/HoodToPitchOffsetDeg", 0.0, true);
	private static final LoggableTunedNumber simMuzzleHeightMeters =
			new LoggableTunedNumber("SimShot/MuzzleHeightM", 0.479, true);
	private static final LoggableTunedNumber simMuzzleForwardMeters =
			new LoggableTunedNumber("SimShot/MuzzleForwardM", 0.18, true);

	protected RebuiltHub blueHub;
	protected RebuiltHub redHub;

	protected RebuiltOutpost blueOutpost;
	protected RebuiltOutpost redOutpost;

	protected boolean isInEfficiencyMode = true;

	protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
	protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
	protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

	private double lastTurretShotTimestampSec = Double.NEGATIVE_INFINITY;
	private int simTurretShotCount = 0;
	public Rebuilt2026FieldSimulation(AbstractDriveTrainSimulation robot) {
		// Initialize faces
		super(robot, new RebuiltFieldObstaclesMap(false));
		blueHub = new RebuiltHub(this, true);
        super.addCustomSimulation(blueHub);

        redHub = new RebuiltHub(this, false);
        super.addCustomSimulation(redHub);

        blueOutpost = new RebuiltOutpost(this, true);
        super.addCustomSimulation(blueOutpost);

        redOutpost = new RebuiltOutpost(this, false);
        super.addCustomSimulation(redOutpost);
	}
	public static double randomInRange(double variance) {
		return (Math.random() - 0.5) * variance;
	}

	public void simulateTurretShotsIfReady() {
		if (!RobotState.isEnabled()) {
			return;
		}
		final double nowSec = Timer.getTimestamp();
		if (nowSec - lastTurretShotTimestampSec < simShotCooldownSec.get()) {
			return;
		}

		final Translation2d hubCenter = getAllianceHubCenter();
		/*if (selectedTurret == null) {
			return;
		}

		spawnTurretShot(selectedTurret, selectedTurret == RobotContainer.leftTurret ? "LeftTurret" : "RightTurret",
				hubCenter);*/
		lastTurretShotTimestampSec = nowSec;
	}

	/*private boolean isTurretReady(Turret turret) {
		return turret.isShotModeActive() && turret.atShootSetpoints();
	}*/

	/*private Turret selectTurretForShot(Translation2d hubCenter, boolean leftReady, boolean rightReady) {
		if (leftReady && !rightReady) {
			return RobotContainer.leftTurret;
		}
		if (!leftReady && rightReady) {
			return RobotContainer.rightTurret;
		}
		if (!leftReady) {
			return null;
		}

		final double leftError = getTurretHeadingErrorToHub(RobotContainer.leftTurret, hubCenter);
		final double rightError = getTurretHeadingErrorToHub(RobotContainer.rightTurret, hubCenter);
		return leftError <= rightError ? RobotContainer.leftTurret : RobotContainer.rightTurret;
	}*/

	/*private double getTurretHeadingErrorToHub(Turret turret, Translation2d hubCenter) {
		final Pose2d robotPose = RobotContainer.drivetrainS.getPose();
		final Pose2d turretPose = robotPose.transformBy(turret.getRobotToTurret());
		final double desiredTurretAngle = hubCenter.minus(turretPose.getTranslation()).getAngle()
				.minus(robotPose.getRotation()).getRadians();
		return Math.abs(MathUtil.angleModulus(desiredTurretAngle - turret.turretAngle()));
	}*/

	/*private void spawnTurretShot(Turret turret, String turretName, Translation2d hubCenter) {
		final Pose2d robotPose = RobotContainer.drivetrainS.getPose();
		final Rotation2d robotHeading = robotPose.getRotation();
		final Rotation2d turretRelativeHeading = new Rotation2d(turret.turretAngle());
		final Rotation2d shotHeading = robotHeading.plus(turretRelativeHeading);

		final double hoodRawRad = turret.hoodAngle();
		double hoodPitchRad =
				(hoodRawRad * simHoodToPitchScale.get()) + Math.toRadians(simHoodToPitchOffsetDeg.get());
		hoodPitchRad = frc.robot.utils.maths.CommonMath.clamp(hoodPitchRad, Math.toRadians(1.0), Math.toRadians(85.0));

		final double flywheelRpm = turret.getCharFlywheelRPM();
		final double launchSpeedMps = Math.max(0.1, flywheelRpm * simBallSpeedMetersPerSecPerRPM.get());
		final double launchHorizontalMps = launchSpeedMps * Math.cos(hoodPitchRad);
		final double launchVerticalMps = launchSpeedMps * Math.sin(hoodPitchRad);

		final double forwardDistance = simMuzzleForwardMeters.get() * Math.cos(hoodPitchRad);
		final Translation2d muzzleOffsetRobot = turret.getRobotToTurret().getTranslation()
				.plus(new Translation2d(forwardDistance, turretRelativeHeading));
		final Translation2d launchPosition = robotPose.getTranslation().plus(muzzleOffsetRobot.rotateBy(robotHeading));
		final double launchHeightMeters = simMuzzleHeightMeters.get() + simMuzzleForwardMeters.get() * Math.sin(hoodPitchRad);

		final ChassisVelocities fieldChassisVelocities = RobotContainer.drivetrainS.getFieldChassisVelocities();
		final Translation2d chassisLinearVelocity = new Translation2d(
				fieldChassisVelocities.vx,
				fieldChassisVelocities.vy);
		final Translation2d chassisRotationalVelocityAtMuzzle = muzzleOffsetRobot.rotateBy(robotHeading)
				.rotateBy(Rotation2d.fromDegrees(90.0))
				.times(fieldChassisVelocities.omega);
		final Translation2d chassisVelocityAtMuzzle =
				chassisLinearVelocity.plus(chassisRotationalVelocityAtMuzzle);
		final Translation2d totalHorizontalVelocity = chassisVelocityAtMuzzle
				.plus(new Translation2d(launchHorizontalMps, shotHeading));

		RebuiltFuelOnFly shot = new RebuiltFuelOnFly(
				launchPosition,
				new Translation2d(),
				new ChassisVelocities(chassisVelocityAtMuzzle.getX(), chassisVelocityAtMuzzle.getY(), 0.0),
				shotHeading,
				Meters.of(launchHeightMeters),
				MetersPerSecond.of(launchSpeedMps),
				Radians.of(hoodPitchRad));
		addGamePiece(shot);
		shot.launch();

		simTurretShotCount++;
		logSimShotTelemetry(
				turretName,
				launchPosition,
				launchHeightMeters,
				totalHorizontalVelocity,
				launchVerticalMps,
				shotHeading,
				hoodRawRad,
				hoodPitchRad,
				flywheelRpm,
				launchSpeedMps,
				hubCenter);
	}*/

	private void logSimShotTelemetry(
			String turretName,
			Translation2d launchPosition,
			double launchHeightMeters,
			Translation2d horizontalVelocity,
			double verticalVelocityMps,
			Rotation2d shotHeading,
			double hoodRawRad,
			double hoodPitchRad,
			double flywheelRpm,
			double launchSpeedMps,
			Translation2d hubCenter) {
		final double hubHeight = FieldConstants.Hub.height;
		final Double descendingHubCrossTime =
				solveDescendingTimeAtHeight(launchHeightMeters, verticalVelocityMps, hubHeight);
		final Double landingTime =
				solveDescendingTimeAtHeight(launchHeightMeters, verticalVelocityMps, Inches.of(3.0).in(Meters));

		Logger.recordOutput("SimShot/Count", simTurretShotCount);
		Logger.recordOutput("SimShot/Last/Turret", turretName);
		Logger.recordOutput("SimShot/Last/FlywheelRPM", flywheelRpm);
		Logger.recordOutput("SimShot/Last/HoodRawDeg", Math.toDegrees(hoodRawRad));
		Logger.recordOutput("SimShot/Last/HoodPitchDeg", Math.toDegrees(hoodPitchRad));
		Logger.recordOutput("SimShot/Last/ShotHeadingDeg", shotHeading.getDegrees());
		Logger.recordOutput("SimShot/Last/LaunchSpeedMps", launchSpeedMps);
		Logger.recordOutput("SimShot/Last/LaunchPose",
				new Pose3d(launchPosition.getX(), launchPosition.getY(), launchHeightMeters, new Rotation3d()));
		Logger.recordOutput("SimShot/Last/LaunchDistanceToHubCenterM", launchPosition.getDistance(hubCenter));

		if (descendingHubCrossTime != null) {
			final Translation2d hubCrossPosition = launchPosition.plus(horizontalVelocity.times(descendingHubCrossTime));
			final double hubCrossDistance = hubCrossPosition.getDistance(hubCenter);
			final Translation2d launchToHub = hubCenter.minus(launchPosition);
			final double launchToHubNorm = launchToHub.getNorm();
			double rangeError = 0.0;
			double lateralError = 0.0;
			if (launchToHubNorm > 1e-6) {
				final Translation2d launchToCross = hubCrossPosition.minus(launchPosition);
				final double along =
						(launchToCross.getX() * launchToHub.getX() + launchToCross.getY() * launchToHub.getY())
								/ launchToHubNorm;
				rangeError = along - launchToHubNorm;
				lateralError =
						(launchToCross.getX() * launchToHub.getY() - launchToCross.getY() * launchToHub.getX())
								/ launchToHubNorm;
			}

			Logger.recordOutput("SimShot/Last/HubHeightDescendingTimeSec", descendingHubCrossTime);
			Logger.recordOutput("SimShot/Last/HubHeightCrossPose",
					new Pose3d(hubCrossPosition.getX(), hubCrossPosition.getY(), hubHeight, new Rotation3d()));
			Logger.recordOutput("SimShot/Last/HubHeightDescendingDistanceToHubCenterM", hubCrossDistance);
			Logger.recordOutput("SimShot/Last/HubHeightRangeErrorM", rangeError);
			Logger.recordOutput("SimShot/Last/HubHeightLateralErrorM", lateralError);
		} else {
			Logger.recordOutput("SimShot/Last/HubHeightDescendingTimeSec", Double.NaN);
			Logger.recordOutput("SimShot/Last/HubHeightDescendingDistanceToHubCenterM", Double.NaN);
			Logger.recordOutput("SimShot/Last/HubHeightRangeErrorM", Double.NaN);
			Logger.recordOutput("SimShot/Last/HubHeightLateralErrorM", Double.NaN);
		}

		if (landingTime != null) {
			final Translation2d landingPosition = launchPosition.plus(horizontalVelocity.times(landingTime));
			Logger.recordOutput("SimShot/Last/LandingTimeSec", landingTime);
			Logger.recordOutput("SimShot/Last/LandingPose",
					new Pose3d(landingPosition.getX(), landingPosition.getY(), Inches.of(3.0).in(Meters), new Rotation3d()));
			Logger.recordOutput("SimShot/Last/LandingDistanceToHubCenterM", landingPosition.getDistance(hubCenter));
		} else {
			Logger.recordOutput("SimShot/Last/LandingTimeSec", Double.NaN);
			Logger.recordOutput("SimShot/Last/LandingDistanceToHubCenterM", Double.NaN);
		}
	}

	private static Double solveDescendingTimeAtHeight(double initialHeight, double initialVerticalVelocity, double targetHeight) {
		final double deltaHeight = targetHeight - initialHeight;
		final double discriminant = initialVerticalVelocity * initialVerticalVelocity
				- 2.0 * FieldConstants.COEFFICIENT_OF_GRAVITY * deltaHeight;
		if (discriminant < 0.0) {
			return null;
		}

		final double sqrtDiscriminant = Math.sqrt(discriminant);
		double t1 = (initialVerticalVelocity - sqrtDiscriminant) / FieldConstants.COEFFICIENT_OF_GRAVITY;
		double t2 = (initialVerticalVelocity + sqrtDiscriminant) / FieldConstants.COEFFICIENT_OF_GRAVITY;
		double descendingTime = Math.max(t1, t2);

		if (descendingTime <= 0.0) {
			return null;
		}

		final double verticalVelocityAtCrossing =
				initialVerticalVelocity - FieldConstants.COEFFICIENT_OF_GRAVITY * descendingTime;
		if (verticalVelocityAtCrossing >= 0.0) {
			final double alternateTime = Math.min(t1, t2);
			if (alternateTime <= 0.0) {
				return null;
			}
			final double alternateVerticalVelocity =
					initialVerticalVelocity - FieldConstants.COEFFICIENT_OF_GRAVITY * alternateTime;
			if (alternateVerticalVelocity >= 0.0) {
				return null;
			}
			descendingTime = alternateTime;
		}

		return descendingTime;
	}

	private Translation2d getAllianceHubCenter() {
		return GeomUtil.apply(new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()));
	}

	public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
			Angle pitch,
			double xVariance,
			double yVariance,
			double yawVariance,
			double speedVariance,
			double pitchVariance) {
		RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
				piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
				new Translation2d(),
				new ChassisVelocities(),
				yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
				height,
				speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
				Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance)));
		addGamePiece(fuelOnFly);
		fuelOnFly.launch();
	}
 @Override
    public void placeGamePiecesOnField(boolean preload) {
		//preload isn't allowed in 2026
        blueOutpost.reset();
        redOutpost.reset();

        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y += isInEfficiencyMode ? 3 : 1) {
                addGamePiece(new FuelOnFieldSimulated(centerPieceBottomRightCorner.plus(
                        new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
            }
        }

        boolean isOnBlue = !MatchState.getAlliance().isEmpty()
                && MatchState.getAlliance().get() == Alliance.BLUE;

        if (isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new FuelOnFieldSimulated(blueDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    addGamePiece(new FuelOnFieldSimulated(redDepotBottomRightCorner.plus(
                            new Translation2d(Inches.of(5.991 * x), Inches.of(5.95 * y)))));
                }
            }
		}
    }
	
	/**
	 * the obstacles on the 2025 competition field
	 */
	public static final class RebuiltFieldObstaclesMap
			extends FieldObstaclesMap {
		public RebuiltFieldObstaclesMap(boolean AddRampCollider) {
			// blue wall
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MIN, FIELD_Y_MAX));

			// red wall
			addBorderLine(new Translation2d(FIELD_X_MAX, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));

			// scoring area walls
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MIN), new Translation2d(FIELD_X_MAX, FIELD_Y_MIN));

			// opposite side
			addBorderLine(new Translation2d(FIELD_X_MIN, FIELD_Y_MAX), new Translation2d(FIELD_X_MAX, FIELD_Y_MAX));

			// blue tower uprights
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(UPRIGHT_OFFSET_FROM_END_WALL, UPRIGHT_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							UPRIGHT_OFFSET_FROM_END_WALL,
							UPRIGHT_OFFSET_FROM_SIDE_WALL + UPRIGHT_Y_SPACING,
							new Rotation2d()));

			// red tower uprights
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));
			addRectangularObstacle(
					UPRIGHT_X_LEN,
					UPRIGHT_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - UPRIGHT_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - UPRIGHT_OFFSET_FROM_SIDE_WALL - UPRIGHT_Y_SPACING,
							new Rotation2d()));

			// blue trench wall
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(TRENCH_WALL_OFFSET_FROM_END_WALL, TRENCH_WALL_OFFSET_FROM_SIDE_WALL, new Rotation2d()));
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							TRENCH_WALL_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));

			// red trench wall
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
							TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));
			addRectangularObstacle(
					TRENCH_WALL_X_LEN,
					TRENCH_WALL_Y_LEN,
					new Pose2d(
							FIELD_X_MAX - TRENCH_WALL_OFFSET_FROM_END_WALL,
							FIELD_Y_MAX - TRENCH_WALL_OFFSET_FROM_SIDE_WALL,
							new Rotation2d()));

			// Colliders to describe the hub plus ramps
			if (AddRampCollider) {

				// blue hub + ramps
				addRectangularObstacle(
						HUB_X_LEN, HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));

				// red hub + ramps
				addRectangularObstacle(
						HUB_X_LEN,
						HUB_Y_LEN + 2.0 * HUB_RAMP_LENGTH,
						new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));

			} else {

				// blue hub
				addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(HUB_X, HUB_Y, new Rotation2d()));

				// red hub
				addRectangularObstacle(HUB_X_LEN, HUB_Y_LEN, new Pose2d(FIELD_X_MAX - HUB_X, HUB_Y, new Rotation2d()));
			}
		}

	}
}
