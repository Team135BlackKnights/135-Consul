package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.GeometryConstants;
import frc.robot.Robot;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.AlgaeBall;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.ReefHeight;
import frc.robot.utils.CompetitionFieldUtils.Simulation.CompetitionFieldSimulation;
import frc.robot.utils.maths.TimeUtil;

import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicReference;

import org.dyn4j.geometry.Geometry;
import org.littletonrobotics.junction.Logger;

/*YEARLYUPDATE: Create an alternate version of this year on year. Leave this file yearly as an example. (algae is a shootable piece, coral is a pick-and-place game piece)*/
/**
 * 
 * a set of game pieces of the 2025 game "Reefscape"
 * 
 * @apiNote Game Piece 1 is the shootable one!
 */
public final class Reefscape2025FieldObjects {
	/**
	 * a static gamepiece one on field it is displayed on the dashboard and
	 * telemetry, but
	 * it does not appear in the simulation. meaning that, it does not have
	 * collision space and isn't involved in intake simulation
	 */
	public static class AlgaeBallOnFieldStatic extends GamePieceInSimulation {
		private Pose3d staticPose;

		public AlgaeBallOnFieldStatic(Pose3d staticPose) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2),
					true);
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		public AlgaeBallOnFieldStatic(Pose3d staticPose, double momentumAngle,
				double momentumMagnitude) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2), momentumAngle, momentumMagnitude,
					true);
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.ALGAE_BALL_HEIGHT;
		}

		@Override
		public Pose3d getPose3d() {
			return staticPose;
		}

		@Override
		public String getTypeName() {
			return "AlgaeBall";
		}

		@Override
		public Pair<Boolean,String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone"); // Static/End scored. Cannot score again.
		}

		@Override
		public int getScoreValue(String scoreType) {
			return FieldConstants.ALGAE_BALL_SCORE; // doesn't change throughout game
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude, double momentumAngle) {
			return null; // unused since deleted.
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			return true;
		}
	}

	/**
	 * a static gamepiece two on field it is displayed on the dashboard and
	 * telemetry, but
	 * it does not appear in the simulation. meaning that, it does not have
	 * collision space and isn't involved in intake simulation
	 */
	public static class ReefscapeCoralOnFieldStatic extends GamePieceInSimulation {
		private Pose3d staticPose;

		public ReefscapeCoralOnFieldStatic(Pose3d staticPose) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), false);
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		public ReefscapeCoralOnFieldStatic(Pose3d staticPose, double momentumAngle,
				double momentumMagnitude) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), momentumAngle,
					momentumMagnitude, false);
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.REEFSCAPE_CORAL_HEIGHT;
		}

		@Override
		public Pose3d getPose3d() {
			return staticPose;
		}

		@Override
		public String getTypeName() {
			return "ReefscapeCoral";
		}

		@Override
		public Pair<Boolean,String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone"); // Static/End scored. Cannot score again.
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 0; // doesn't GET scored.
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			return true; // doesn't GET scored.
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude, double momentumAngle) {
			return null;
		}
	}

	/**
	 * a simulated game piece one on field has collision space, and can be "grabbed"
	 * by an
	 * intake simulation
	 */
	public static class AlgaeBallOnFieldSimulated extends GamePieceInSimulation {
		public AlgaeBallOnFieldSimulated(Translation2d initialPosition) {
			super(initialPosition, Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2), true);
		}

		public AlgaeBallOnFieldSimulated(Translation2d initialPosition, double momentumAngle,
				double momentumMagnitude) {
			super(initialPosition, Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2), momentumAngle,
					momentumMagnitude, true);
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.ALGAE_BALL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "AlgaeBall";
		}

		@Override
		public Pair<Boolean,String> isInScoreZone() {
			Translation3d position = getPose3d().getTranslation();
			if (position.getY() >= FieldConstants.FIELD_HEIGHT + .05
					|| position.getY() <= -.05) {
				return new Pair<>(false, "Processor");
			}
			return new Pair<>(false, "NotInScoreZone");
		}

		@Override
		public int getScoreValue(String scoreType) {
			return FieldConstants.ALGAE_BALL_SCORE; // doesn't change throughout game
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			return true;
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldScore, double magnitude, double momentumAngle) {
			return null;
		}
	}

	/**
	 * a simulated game piece one on field has collision space, and can be "grabbed"
	 * by an
	 * intake simulation
	 */
	public static class ReefscapeCoralOnFieldSimulated extends GamePieceInSimulation {
		public ReefscapeCoralOnFieldSimulated(Translation2d initialPosition) {
			super(initialPosition, Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), false);
		}

		public ReefscapeCoralOnFieldSimulated(Translation2d initialPosition, double momentumAngle,
				double momentumMagnitude) {
			super(initialPosition, Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), momentumAngle,
					momentumMagnitude, false);
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.REEFSCAPE_CORAL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "ReefscapeCoral";
		}

		@Override
		public Pair<Boolean,String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone");
		}
		@Override
		public int getScoreValue(String scoreType) {
			return 0; // doesn't GET scored.
		}
		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			return true; // doesn't GET scored.
		}
		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldScore, double magnitude, double momentumAngle) {
			return null; // unused since deleted.
		}
	}

	/**
	 * an algae that is locked on to the manipulator's position for the algae
	 */
	public static class AlgaeBallOnManipulator extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private Pose3d currentPose;
		private final Pose3d startingPose;
		private double totalTimeSec;

		public AlgaeBallOnManipulator(double launchingTimeStampSec,
				double launchingSpeedMetersPerSec, Pose3d currentPose) {
			super(new Translation2d(), Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2), false);
			super.setEnabled(false);
			this.currentPose = currentPose;
			this.startingPose = currentPose;
			this.launchingTimeStampSec = launchingTimeStampSec;
			// Calculate the total time to reach the manipulator
			this.totalTimeSec = startingPose.getTranslation()
					.getDistance(RobotContainer.fieldSimulation
							.getMainDriveSimulation().getPose3d()
							.transformBy(GeometryConstants.coralScorerTransform).getTranslation())
					/ launchingSpeedMetersPerSec * 1e6;
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.ALGAE_BALL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "AlgaeBall";
		}

		@Override
		public Pose3d getPose3d() {
			double currentTime = Logger.getTimestamp();
			// set the pose's rotation
			Pose3d manipulatorPose3d = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d().plus(new Transform3d(
							Units.inchesToMeters(2), Units.inchesToMeters(0), Units.inchesToMeters(6),
							new Rotation3d()));
			// manipulatorPose3d = new Pose3d(manipulatorPose3d.getTranslation(),
			// new
			// Rotation3d(0,-RobotContainer.armS.getDistance(),RobotContainer.drivetrainS.getPose().getRotation().getRadians()));
			if ((currentTime - launchingTimeStampSec) > (launchingTimeStampSec
					+ totalTimeSec)) {
				return manipulatorPose3d;
			}
			double timeProportion = (currentTime - launchingTimeStampSec)
					/ totalTimeSec;
			currentPose = startingPose.interpolate(manipulatorPose3d,
					timeProportion);
			return currentPose;
		}

		@Override
		public Pair<Boolean, String> isInScoreZone() {
			//This went unimplemented. It SHOULD have logic. Nah. -G
			return new Pair<>(false, "NotInScoreZone"); 
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 0; // doesn't change throughout game
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			// This went unimplemented. It SHOULD have logic. Nah. -G
			return false; // never delete, it is always on the manipulator
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude,
				double momentumAngle) {
			return null;
		}
	}

	/**
	 * a coral that is locked on to the manipulator's position for the game
	 * piece two
	 */
	public static class ReefscapeCoralOnManipulator extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private Pose3d currentPose = new Pose3d();
		private double totalTimeSec = 1;

		/**
		 * Runs an animation to get to the manipulator pose.
		 */
		public ReefscapeCoralOnManipulator() {
			super(new Translation2d(), Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), false);
			super.setEnabled(false);
			this.launchingTimeStampSec = TimeUtil.getLogTimeSeconds();
			currentPose = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d().plus(GeometryConstants.hopperStartTransform);// an exact
																										// offset point
																										// from robot
																										// center
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.REEFSCAPE_CORAL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "ReefscapeCoral";
		}

		@Override
		public Pose3d getPose3d() {
			double currentTime = TimeUtil.getLogTimeSeconds();
			// set the pose's rotation
			Pose3d manipulatorPose3d = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d()
					.plus(GeometryConstants.coralScorerTransform.plus(new Transform3d(Units.inchesToMeters(-1),
							Units.inchesToMeters(-6) + .05, Units.inchesToMeters(23.45) - .5, new Rotation3d())));
			Pose3d angledHopper = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d()
					.plus(GeometryConstants.hopperTransform);
			Pose3d angledMiddleHopper = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d()
					.plus(GeometryConstants.hopperMiddleTransform);
			if ((currentTime - launchingTimeStampSec) > (launchingTimeStampSec // 28.5 - 27.5 > 27.5+1
					+ totalTimeSec)) {
				// if (!lockedIn){
				// lockedIn = true;
				// }
				return manipulatorPose3d;
			}
			// go to angle pose
			if (currentTime - launchingTimeStampSec < 0.25) {
				double timeProportion = (currentTime - launchingTimeStampSec)
						/ .25;
				currentPose = currentPose.interpolate(angledMiddleHopper, timeProportion);
			} else if (currentTime - launchingTimeStampSec < .5) {
				double timeProportion = (currentTime - (launchingTimeStampSec + .25))
						/ .25;
				currentPose = currentPose.interpolate(angledHopper, timeProportion);
			}

			else {
				double timeProportion = (currentTime - (launchingTimeStampSec + 0.5)) / .5;
				currentPose = currentPose.interpolate(manipulatorPose3d, timeProportion);

			}

			return currentPose;
		}

		@Override
		public Pair<Boolean, String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone");
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 0;
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			return false; // never delete
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude,
				double momentumAngle) {
			return null;
		}
	}

	/**
	 * a game piece one that is flying from a shooter to the scoring location
	 * the flight is simulated by a simple linear animation
	 */
	public static class AlgaeBallInFly extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private Pose3d currentPose;
		private final Pose3d startingPose;

		public AlgaeBallInFly(double launchingTimeStampSec,
				double launchingSpeedMetersPerSec, Pose3d startingPose) {
			super(startingPose.toPose2d().getTranslation(),
					Geometry.createCircle(FieldConstants.ALGAE_BALL_DIAMETER / 2), true);
			super.setEnabled(true);
			this.currentPose = startingPose;
			this.startingPose = startingPose;
			this.launchingTimeStampSec = launchingTimeStampSec;
			this.momentumAngle = startingPose.getRotation().getZ() + Math.PI;
			this.momentumMagnitude = launchingSpeedMetersPerSec;
		}

		@Override
		public String getTypeName() {
			return "AlgaeBall";
		}

		@Override
		public Pose3d getPose3d() {
			double deltaTSeconds = Math
					.abs(TimeUtil.getLogTimeSeconds() - launchingTimeStampSec);
			// To visualize the math
			double vNoughtZ = momentumMagnitude
					* Math.sin(startingPose.getRotation().getY());
			double vNoughtX = -momentumMagnitude
					* Math.cos(startingPose.getRotation().getY());
			double expDecay = Math.pow(Math.E, -deltaTSeconds / AlgaeBall.M_OVER_K);
			double updatedPosZMeters = AlgaeBall.M_OVER_K
					* (vNoughtZ + AlgaeBall.M_OVER_K * FieldConstants.COEFFICIENT_OF_GRAVITY) * (1 - expDecay)
					- (AlgaeBall.M_OVER_K * FieldConstants.COEFFICIENT_OF_GRAVITY * deltaTSeconds);
			double updatedPosYMeters = 0;
			double updatedPosXMeters = AlgaeBall.M_OVER_K * vNoughtX * (1 - expDecay);
			double updatedPitch = Math.atan2(updatedPosZMeters, Math.hypot(updatedPosXMeters, updatedPosYMeters));
			Transform3d projectileTranslationVector = new Transform3d(
					updatedPosXMeters, updatedPosYMeters, updatedPosZMeters,
					new Rotation3d(0, updatedPitch, momentumAngle - startingPose.getRotation().getZ() + Math.PI));
			// Update the current pose based on the projectile's new position
			currentPose = startingPose.plus(projectileTranslationVector);
			momentumAngle = currentPose.getRotation().getZ() + Math.PI;
			return currentPose;
		}

		@Override
		public Pose2d getObjectOnFieldPose2d() {
			return getPose3d().toPose2d();
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.ALGAE_BALL_HEIGHT;
		}

		@Override
		public Pair<Boolean, String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone"); // flying, not scored yet
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 0; // not scored
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			// if the game piece is not in the air anymore, delete it
			return false; // not deleted yet
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude,
				double momentumAngle) {
			return null;
		}
	}

	/**
	 * a game piece two that is flying from a shooter to the scoring location
	 * the flight is simulated by a simple linear animation
	 */
	public static class ReefscapeCoralInFly extends GamePieceInSimulation {
		private double lastUpdate;
		private Pose3d currentPose;
		private final Pose3d startingPose;
		private double velocityX = 0,
				velocityY = 0,
				velocityZ = 0, updatedPitch = 0;

		public ReefscapeCoralInFly(double launchingTimeStampSec, Pose3d startingPose) {
			super(startingPose.toPose2d().getTranslation(),
					Geometry.createCircle(FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2), false);
			super.setEnabled(false);
			this.currentPose = startingPose;
			this.startingPose = startingPose;
			this.lastUpdate = launchingTimeStampSec;
			this.momentumAngle = startingPose.getRotation().getZ();
		}

		@Override
		public String getTypeName() {
			return "ReefscapeCoral";
		}

		@Override
		public Pose3d getPose3d() {
			double currentTime = TimeUtil.getLogTimeSeconds();
			double deltaTSeconds = Math
					.abs(currentTime - lastUpdate);
			lastUpdate = currentTime;
			// if WITHIN robot, fall on a slope (inclined plane)
			if ((Math.abs(currentPose.getTranslation().getDistance(startingPose.getTranslation())) < Units
					.inchesToMeters(4))) {
				velocityZ -= FieldConstants.COEFFICIENT_OF_GRAVITY * deltaTSeconds
						* Math.sin(currentPose.getRotation().getY());
				double velocityXY = FieldConstants.COEFFICIENT_OF_GRAVITY * deltaTSeconds
						* Math.cos(currentPose.getRotation().getY())
						+ CompetitionFieldSimulation.calculateObjectSpeed(RobotContainer.drivetrainS.getFieldVelocity(),
								100);
				velocityX = velocityXY * Math.cos(currentPose.getRotation().getZ());
				velocityY = velocityXY * Math.sin(currentPose.getRotation().getZ());
				Logger.recordOutput("StillIn", true);

			} else {
				// if the coral has left the robot, assume it falls with the velocities that it
				// had when it left the robot (and gravity)
				velocityZ -= FieldConstants.COEFFICIENT_OF_GRAVITY * deltaTSeconds;
				Logger.recordOutput("StillIn", false);
				updatedPitch = Math.atan2(velocityZ, Math.hypot(velocityX, velocityY));
			}
			double vectorX = velocityX * deltaTSeconds;
			double vectorY = velocityY * deltaTSeconds;
			double vectorZ = velocityZ * deltaTSeconds;
			Translation3d vector = new Translation3d(vectorX, vectorY, vectorZ);
			momentumAngle = startingPose.getRotation().getZ();
			momentumMagnitude = vector.getNorm();
			currentPose = new Pose3d(currentPose.getTranslation().plus(vector),
					currentPose.getRotation().plus(new Rotation3d(0, -updatedPitch * deltaTSeconds, 0)));
			return currentPose;
		}

		@Override
		public Pose2d getObjectOnFieldPose2d() {
			return getPose3d().toPose2d();
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.REEFSCAPE_CORAL_HEIGHT;
		}

		private Pair<Pose3d, ReefHeight> getNearestScoringLocationCoral(Pose3d position) {
			AtomicReference<Pose3d> closestScoringPose = new AtomicReference<>(new Pose3d());
			if (Robot.isRed == false) {
				FieldConstants.Reef.blueBranchPositions.forEach((entry) -> {
					for (Entry<ReefHeight, Pose3d> entries : entry.entrySet()) {
						if (entries.getValue()
								.relativeTo(
										position.plus(
											new Transform3d(Units.inchesToMeters(6.2), 0, 0,
											new Rotation3d())))
								.getTranslation()
								.getNorm() < closestScoringPose.get()
										.relativeTo(position
												.plus(new Transform3d(Units.inchesToMeters(6.2), 0, 0,
												new Rotation3d())))
										.getTranslation()
										.getNorm()) {
							closestScoringPose.set(entries.getValue());
						}
					}
				});
			} else {
				FieldConstants.Reef.redBranchPositions.forEach((entry) -> {
					for (Entry<ReefHeight, Pose3d> entries : entry.entrySet()) {
						if (entries.getValue()
								.relativeTo(
										position.plus(
											new Transform3d(Units.inchesToMeters(6.2), 0, 0,
											new Rotation3d())))
								.getTranslation()
								.getNorm() < closestScoringPose.get()
										.relativeTo(position
												.plus(new Transform3d(Units.inchesToMeters(6.2), 0, 0,
												new Rotation3d())))
										.getTranslation()
										.getNorm()) {
							closestScoringPose.set(entries.getValue());
						}
					}
				});
			}
			if (closestScoringPose.get() != null) {
				// Logger.recordOutput("ClosestScoreingSpot", closestScoringPose.get());
				if (closestScoringPose.get().getZ() > 1.6) {
					// System.out.println("L4");
					// Logger.recordOutput("ClosestScoringSpot", closestScoringPose.get().plus(new
					// Transform3d(.06,0,0,new Rotation3d())));
					// Logger.recordOutput("HitPoint",
					// position.plus(GeometryConstants.ReefscapeGeometryScoring.CoralDistanceFromCenter));
					if (closestScoringPose.get().plus(new Transform3d(.06, 0, 0, new Rotation3d()))
							.relativeTo(
									position.plus(new Transform3d(Units.inchesToMeters(6.2), 0, 0,
									new Rotation3d())))
							.getTranslation()
							.getNorm() < FieldConstants.SCORING_COLLISION_RADIUS_REEFSCAPE_CORAL) {
						if (closestScoringPose.get().getZ() < FieldConstants.ReefHeight.L3.height) {
							Pose3d pose = closestScoringPose.get();
							return new Pair<>(
									new Pose3d(pose.getTranslation(),
											new Rotation3d(pose.getRotation().getX(),
													FieldConstants.ReefHeight.L2.pitch,
													pose.getRotation().getZ())),
									FieldConstants.ReefHeight.L2);
						} else if (closestScoringPose.get().getZ() < FieldConstants.ReefHeight.L4.height) {
							Pose3d pose = closestScoringPose.get();
							return new Pair<>(
									new Pose3d(pose.getTranslation(),
											new Rotation3d(pose.getRotation().getX(),
													FieldConstants.ReefHeight.L3.pitch,
													pose.getRotation().getZ())),
									FieldConstants.ReefHeight.L3);
						} else {
							Pose3d pose = closestScoringPose.get();
							return new Pair<>(
									new Pose3d(pose.getTranslation(),
											new Rotation3d(pose.getRotation().getX(),
													FieldConstants.ReefHeight.L4.pitch,
													pose.getRotation().getZ())),
									FieldConstants.ReefHeight.L4);
						}
					}
				}
				if (closestScoringPose.get()
						.relativeTo(position.plus(new Transform3d(Units.inchesToMeters(6.2), 0, 0,
						new Rotation3d())))
						.getTranslation()
						.getNorm() < FieldConstants.SCORING_COLLISION_RADIUS_REEFSCAPE_CORAL) {
					if (closestScoringPose.get().getZ() < FieldConstants.ReefHeight.L3.height) {
						Pose3d pose = closestScoringPose.get();
						return new Pair<>(
								new Pose3d(pose.getTranslation(),
										new Rotation3d(pose.getRotation().getX(), FieldConstants.ReefHeight.L2.pitch,
												pose.getRotation().getZ())),
								FieldConstants.ReefHeight.L2);
					} else if (closestScoringPose.get().getZ() < FieldConstants.ReefHeight.L4.height) {
						Pose3d pose = closestScoringPose.get();
						return new Pair<>(
								new Pose3d(pose.getTranslation(),
										new Rotation3d(pose.getRotation().getX(), FieldConstants.ReefHeight.L3.pitch,
												pose.getRotation().getZ())),
								FieldConstants.ReefHeight.L3);
					} else {
						Pose3d pose = closestScoringPose.get();
						return new Pair<>(
								new Pose3d(pose.getTranslation(),
										new Rotation3d(pose.getRotation().getX(), FieldConstants.ReefHeight.L4.pitch,
												pose.getRotation().getZ())),
								FieldConstants.ReefHeight.L4);
					}
				} else {
					return null;
				}

			}
			return null;

		}

		@Override
		public Pair<Boolean, String> isInScoreZone() {
			Pair<Pose3d, ReefHeight> scoringLocation = getNearestScoringLocationCoral(getPose3d());
			if (scoringLocation != null) {
				return new Pair<>(true, "ReefScore");
			}
			if (getPose3d().getTranslation()
					.getZ() <= Units.inchesToMeters(18)) { // collision with ground
				// make the gamepiece a ground note
				// check if the note is close enough to a speaker
				// otherwise, make it a ground note
				Pose3d pose = getPose3d();
				if (pose.getTranslation().toTranslation2d().getDistance(new Translation2d(
						FieldConstants.blueCenterX, FieldConstants.blueCenterY)) < FieldConstants.radius) {
					return new Pair<>(true, "L1Score");
				}
				if (pose.getTranslation().getZ() <= 0.03) {
					return new Pair<>(true, "GroundScore");
				}
			}
			return new Pair<>(false, "NotInScoreZone");
		}

		@Override
		public int getScoreValue(String scoreType) {
			if (scoreType.equals("ReefScore")) {
				Pair<Pose3d, ReefHeight> scoringLocation = getNearestScoringLocationCoral(getPose3d());
				if (scoringLocation != null) {
					// if it is, make it a speaker note
					switch (scoringLocation.getSecond()) {
						case L1:
							if (Constants.currentMatchState == FRCMatchState.AUTO) {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L1_AUTO;
							} else {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L1_TELE;
							}

						case L2:
							if (Constants.currentMatchState == FRCMatchState.AUTO) {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L2_AUTO;
							} else {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L2_TELE;
							}

						case L3:
							if (Constants.currentMatchState == FRCMatchState.AUTO) {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L3_AUTO;
							} else {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L3_TELE;
							}
						case L4:
							if (Constants.currentMatchState == FRCMatchState.AUTO) {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L4_AUTO;
							} else {
								return FieldConstants.REEFSCAPE_CORAL_SCORE_L4_TELE;
							}

					}
				}
			}
			if (scoreType.equals("L1Score")) {
				if (Constants.currentMatchState == FRCMatchState.AUTO) {
					return FieldConstants.REEFSCAPE_CORAL_SCORE_L1_AUTO;
				} else {
					return FieldConstants.REEFSCAPE_CORAL_SCORE_L1_TELE;
				}
			}
			if (scoreType.equals("GroundScore")) {
				return 0;
			}
			return -1; // you deserve to lose points for messing up the scoring system.
		}

		@Override
		public boolean shouldDeleteAndRemoveFromSimulation(String scoreType) {
			if (scoreType.equals("ReefScore") || scoreType.equals("GroundScore")){
				return false;
			}
			return true;
		}

		@Override
		public GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude, double momentumAngle) {
			if (scoreType.equals("ReefScore")){
				Pair<Pose3d, ReefHeight> scoringLocation = getNearestScoringLocationCoral(getPose3d());
				return new ReefscapeCoralOnFieldStatic(scoringLocation.getFirst(), momentumAngle, magnitude);
			}
			//we are ground scoring.
			if (scoreType.equals("GroundScore")) {
				// return a static game piece on field
				return new ReefscapeCoralOnFieldSimulated(oldPose.getTranslation().toTranslation2d(), momentumAngle, magnitude);
			}
			return null; //should never get here.
		}
	}
}
