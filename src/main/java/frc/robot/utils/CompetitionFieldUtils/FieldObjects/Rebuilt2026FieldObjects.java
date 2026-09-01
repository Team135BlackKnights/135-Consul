package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import org.wpilib.math.util.Pair;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import frc.robot.RobotContainer;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.Fuel;
import static org.wpilib.units.Units.*;

import org.dyn4j.geometry.Geometry;
import org.littletonrobotics.junction.Logger;

/*YEARLYUPDATE: Create an alternate version of this year on year. Leave this file yearly as an example. (algae is a shootable piece, coral is a pick-and-place game piece)*/
/**
 * 
 * a set of game pieces of the 2025 game "Reefscape"
 * 
 * @apiNote Game Piece 1 is the shootable one!
 */
public final class Rebuilt2026FieldObjects {
	/**
	 * a static gamepiece one on field it is displayed on the dashboard and
	 * telemetry, but
	 * it does not appear in the simulation. meaning that, it does not have
	 * collision space and isn't involved in intake simulation
	 */
	public static class RebuiltFuelOnFieldStatic extends GamePieceInSimulation {
		private Pose3d staticPose;

		public RebuiltFuelOnFieldStatic(Pose3d staticPose) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2));
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		public RebuiltFuelOnFieldStatic(Pose3d staticPose, double momentumAngle,
				double momentumMagnitude) {
			super(staticPose.getTranslation().toTranslation2d(),
					Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2), momentumAngle, momentumMagnitude);
			super.setEnabled(false);
			this.staticPose = staticPose;
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.FUEL_HEIGHT;
		}

		@Override
		public Pose3d getPose3d() {
			return staticPose;
		}

		@Override
		public String getTypeName() {
			return "Fuel";
		}
@Override
    public boolean isGrounded() {
        return true;
    }
		@Override
		public Pair<Boolean,String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone"); // Static/End scored. Cannot score again.
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 1; // doesn't change throughout game
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
	 * a simulated game piece one on field has collision space, and can be "grabbed"
	 * by an
	 * intake simulation
	 */
	public static class FuelOnFieldSimulated extends GamePieceInSimulation {
		public FuelOnFieldSimulated(Translation2d initialPosition) {
			super(initialPosition, Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2));
		}

		public FuelOnFieldSimulated(Translation2d initialPosition, double momentumAngle,
				double momentumMagnitude) {
			super(initialPosition, Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2), momentumAngle,
					momentumMagnitude);
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.FUEL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "Fuel";
		}
@Override
    public boolean isGrounded() {
        return true;
    }
		@Override
		public Pair<Boolean,String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone");
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 1; // doesn't change throughout game
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
	public static class FuelOnManipulator extends GamePieceInSimulation {
		private final double launchingTimeStampSec;
		private Pose3d currentPose;
		private final Pose3d startingPose;
		private double totalTimeSec;

		public FuelOnManipulator(double launchingTimeStampSec,
				double launchingSpeedMetersPerSec, Pose3d currentPose) {
			super(new Translation2d(), Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2));
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
			return FieldConstants.FUEL_HEIGHT;
		}

		@Override
		public String getTypeName() {
			return "Fuel";
		}
		@Override
    public boolean isGrounded() {
        return true;
    }
		@Override
		public boolean isVisibleToSimObjectDetection() {
			return false;
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
			double elapsedTime = currentTime - launchingTimeStampSec;
			double safeTotalTime = Math.max(totalTimeSec, 1.0);
			if (elapsedTime >= safeTotalTime) {
				return manipulatorPose3d;
			}
			double timeProportion = Math.max(0.0,
					Math.min(1.0, elapsedTime / safeTotalTime));
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
	 * a game piece one that is flying from a shooter to the scoring location
	 * the flight is simulated by a simple linear animation
	 */
	public static class RebuiltFuelOnFly extends GamePieceInSimulation{
		public RebuiltFuelOnFly(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisVelocities chassisSpeedsFieldRelative,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity launchingSpeed,
            Angle shooterAngle) {
        super(Geometry.createCircle(FieldConstants.FUEL_DIAMETER / 2), Fuel.DEFAULT_MASS_KG,
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeedsFieldRelative,
                shooterFacing,
                initialHeight,
                launchingSpeed,
                shooterAngle);

        super.withTouchGroundHeight(Inches.of(3).in(Meters));
        super.enableBecomesGamePieceOnFieldAfterTouchGround();
    }

		@Override
		public String getTypeName() {
			return "Fuel";
		}
@Override
    public boolean isGrounded() {
        return false;
    }
		@Override
		public Pose3d getPose3d() {
			//TODO
			return super.getPose3d();
		}

		@Override
		public Pose2d getObjectOnFieldPose2d() {
			return getPose3d().toPose2d();
		}

		@Override
		public double getGamePieceHeight() {
			return FieldConstants.FUEL_HEIGHT;
		}

		@Override
		public Pair<Boolean, String> isInScoreZone() {
			return new Pair<>(false, "NotInScoreZone"); // flying, not scored yet
		}

		@Override
		public int getScoreValue(String scoreType) {
			return 1; // not scored
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
}
