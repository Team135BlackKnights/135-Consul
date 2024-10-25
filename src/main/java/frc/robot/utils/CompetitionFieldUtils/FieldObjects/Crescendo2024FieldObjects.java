package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.CrescendoNote;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePieceTag;
import frc.robot.utils.maths.TimeUtil;

import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

/**
 * a set of game pieces of the 2024 game "Crescendo"
 */
public final class Crescendo2024FieldObjects {
	/* https://www.andymark.com/products/frc-2024-am-4999 */
	private static final double NOTE_HEIGHT = Units.inchesToMeters(2),
			NOTE_DIAMETER = Units.inchesToMeters(14);

	/**
	 * a static note on field it is displayed on the dashboard and telemetry, but
	 * it does not appear in the simulation. meaning that, it does not have
	 * collision space and isn't involved in intake simulation
	 */
	public static class NoteOnFieldStatic implements GamePieceOnFieldDisplay {
		private final Translation2d initialPosition;

		public NoteOnFieldStatic(Translation2d initialPosition) {
			this.initialPosition = initialPosition;
		}

		@Override
		public Pose2d getObjectOnFieldPose2d() {
			return new Pose2d(initialPosition, new Rotation2d());
		}

		@Override
		public String getTypeName() { return "Note"; }

		@Override
		public double getGamePieceHeight() { return NOTE_HEIGHT; }
	}

	/**
	 * a simulated note on field has collision space, and can be "grabbed" by an
	 * intake simulation
	 */
	public static class NoteOnFieldSimulated extends GamePieceInSimulation {
		public NoteOnFieldSimulated(Translation2d initialPosition,
				Vector2 speedVector) {
			super(initialPosition, Geometry.createCircle(NOTE_DIAMETER / 2),
					GamePieceTag.ON_GROUND);
		}

		public NoteOnFieldSimulated(Translation2d initialPosition,
				double momentumAngle, double momentumMagnitude) {
			super(initialPosition, Geometry.createCircle(NOTE_DIAMETER / 2),
					GamePieceTag.ON_GROUND, momentumAngle, momentumMagnitude);
		}

		@Override
		public double getGamePieceHeight() { return NOTE_HEIGHT; }

		@Override
		public String getTypeName() { return "Note"; }
	}

	/**
	 * a note that is locked on to the manipulator's position for the note
	 */
	public static class NoteOnManipulator extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private final Transform3d manipulatorTransform3d; // The manipulator's position
		private Pose3d currentPose;
		private final Pose3d startingPose;
		private double totalTimeSec;

		public NoteOnManipulator(double launchingTimeStampSec,
				double launchingSpeedMetersPerSec, Pose3d currentPose,
				Transform3d manipulatorTransform) {
			super(RobotContainer.fieldSimulation.getMainDriveSimulation()
					.getPose3d().transformBy(manipulatorTransform).getTranslation()
					.toTranslation2d(), Geometry.createCircle(NOTE_DIAMETER / 2),
					GamePieceTag.IN_ROBOT);
			super.setEnabled(false);
			this.currentPose = currentPose;
			this.startingPose = currentPose;
			this.launchingTimeStampSec = launchingTimeStampSec;
			this.manipulatorTransform3d = manipulatorTransform;
			// Calculate the total time to reach the manipulator
			this.totalTimeSec = startingPose.getTranslation()
					.getDistance(RobotContainer.fieldSimulation
							.getMainDriveSimulation().getPose3d()
							.transformBy(manipulatorTransform).getTranslation())
					/ launchingSpeedMetersPerSec * 1e6;
			System.out.println("totalTimeSec: " + totalTimeSec / 1e6);
		}

		@Override
		public double getGamePieceHeight() { return NOTE_HEIGHT; }

		@Override
		public String getTypeName() { return "Note"; }

		@Override
		public Pose3d getPose3d() {
			double currentTime = Logger.getTimestamp();
			Pose3d manipulatorPose3d = RobotContainer.fieldSimulation
					.getMainDriveSimulation().getPose3d()
					.transformBy(manipulatorTransform3d);
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
	}

	/**
	 * a note that is flying from a shooter to the speaker the flight is
	 * simulated by a simple linear animation
	 */
	public static class NoteInFly extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private Pose3d currentPose;
		private final Pose3d startingPose;

		public NoteInFly(double launchingTimeStampSec,
				double launchingSpeedMetersPerSec, Pose3d startingPose) {
			super(startingPose.toPose2d().getTranslation(),
					Geometry.createCircle(NOTE_DIAMETER / 2), GamePieceTag.IN_AIR);
			super.setEnabled(true);
			this.currentPose = startingPose;
			this.startingPose = startingPose;
			this.launchingTimeStampSec = launchingTimeStampSec;
			this.momentumAngle = startingPose.getRotation().getZ() + Math.PI;
			this.momentumMagnitude = launchingSpeedMetersPerSec;
		}

		@Override
		public String getTypeName() { return "Note"; }

		@Override
		public Pose3d getPose3d() {
			double deltaTSeconds = Math
					.abs(TimeUtil.getLogTimeSeconds() - launchingTimeStampSec);
			//To visualize the math 
			double vNoughtZ = momentumMagnitude * Math.sin(
					startingPose.getRotation().getY() - Units.degreesToRadians(25));
			double vNoughtX = -momentumMagnitude * Math.cos(
					startingPose.getRotation().getY() - Units.degreesToRadians(25));
			double expDecay = Math.pow(Math.E,
					-deltaTSeconds / CrescendoNote.M_OVER_K);
			double updatedPosZMeters = CrescendoNote.M_OVER_K
					* (vNoughtZ + CrescendoNote.M_OVER_K
							* FieldConstants.COEFFICIENT_OF_GRAVITY)
					* (1 - expDecay)
					- (CrescendoNote.M_OVER_K * FieldConstants.COEFFICIENT_OF_GRAVITY
							* deltaTSeconds);
			double updatedPosYMeters = 0;
			double updatedPosXMeters = CrescendoNote.M_OVER_K * vNoughtX
					* (1 - expDecay);
			double updatedPitch = Math.atan2(updatedPosZMeters,
					Math.hypot(updatedPosXMeters, updatedPosYMeters));
			Transform3d projectileTranslationVector = new Transform3d(
					updatedPosXMeters, updatedPosYMeters, updatedPosZMeters,
					new Rotation3d(0, updatedPitch, momentumAngle
							- startingPose.getRotation().getZ() + Math.PI));
			// Update the current pose based on the projectile's new position
			currentPose = startingPose.plus(projectileTranslationVector);
			momentumAngle = currentPose.getRotation().getZ() + Math.PI;
			return currentPose;
		}

		@Override
		public Pose2d getObjectOnFieldPose2d() { return getPose3d().toPose2d(); }

		@Override
		public double getGamePieceHeight() { return NOTE_HEIGHT; }
	}
}
