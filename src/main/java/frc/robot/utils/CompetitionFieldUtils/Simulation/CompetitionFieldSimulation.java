package frc.robot.utils.CompetitionFieldUtils.Simulation;

import org.wpilib.math.util.Pair;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Rebuilt2026FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedBattery;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePiece;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.CompetitionFieldUtils.CompField;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.maths.GeometryConvertor;
import frc.robot.utils.maths.TimeUtil;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.ContactConstraint;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Set;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

//YEARLYUPDATE: change this to match the year
/**
 * this class simulates the physical behavior of all the objects on field should
 * only be created during a robot simulation (not in real or replay mode)
 */
@SuppressWarnings("unused")
public abstract class CompetitionFieldSimulation {
	private final World<Body> physicsWorld;
	private final CompField competitionField;
	private final static Set<AbstractDriveTrainSimulation> robotSimulations = new HashSet<>();
	private final Set<Simulatable> customSims;
	private final AbstractDriveTrainSimulation mainRobot;
	private final static Set<GamePieceInSimulation> gamePieces = new HashSet<>();
	private static double score = 0, scoredCoral = 0, scoredAlgae = 0;

	public interface Simulatable {
		/**
		 * Called in {@link #simulationSubTick(int)}.
		 *
		 * @param subTickNum the number of this sub-tick (counting from 0 in each robot
		 *                   period)
		 */
		void simulationSubTick(int subTickNum);
	}

	public CompetitionFieldSimulation(AbstractDriveTrainSimulation mainRobot,
			FieldObstaclesMap obstaclesMap) {
		this.competitionField = new CompField(mainRobot);
		this.mainRobot = mainRobot;
		this.physicsWorld = new World<>();
		customSims = new HashSet<>();
		this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
		for (Body obstacle : obstaclesMap.obstacles)
			this.physicsWorld.addBody(obstacle);
		this.physicsWorld.addBody(mainRobot);
		robotSimulations.add(mainRobot);
	}

	public void addPoints(int points) {
		score += points;
		Logger.recordOutput("Field/Scoring/SimScore", score);
	}

	public synchronized void addCustomSimulation(Simulatable simulatable) {
		this.customSims.add(simulatable);
	}

	public void updateSimulationWorld() {
		final double subPeriodSeconds = Robot.defaultPeriodSecs
				/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
		// move through 5 sub-periods in each update
		for (int i = 0; i < DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD; i++) {
			try {
				this.physicsWorld.step(1, subPeriodSeconds);
			} catch (Exception e) {
				System.out.println("Physics World Step Failed, trying one more time");
				this.physicsWorld.step(1, subPeriodSeconds);
			}
			// update the battery
			SimulatedBattery.simulationSubTick();
			for (AbstractDriveTrainSimulation robotSimulation : robotSimulations)
				robotSimulation.simulationSubTick();
			for (Simulatable customSim : customSims)
				customSim.simulationSubTick(i);
			// go through all game pieces
			Set<GamePieceInSimulation> gamePiecesCopy = new HashSet<>(gamePieces); // Create a copy of the gamePieces
																					// set
			for (GamePieceInSimulation gamePiece : gamePiecesCopy) { // Iterate over the copy
				// Handle airborne projectiles before any other interactions.
				if (!gamePiece.isGrounded()) {
					if (gamePiece.hasHitTarget()) {
						gamePiece.triggerHitTargetCallBack();
						removeGamepiece(gamePiece.cleanUp());
						continue;
					}

					final boolean hitGround = gamePiece.hasHitGround();
					final boolean outOfField = gamePiece.hasGoneOutOfField();
					if (hitGround || outOfField) {
						final Pose3d settledPose = gamePiece.getPose3d();
						final boolean shouldBecomeGroundPiece = hitGround
								&& gamePiece.shouldBecomeGamePieceOnFieldAfterTouchGround();
						removeGamepiece(gamePiece.cleanUp());

						if (shouldBecomeGroundPiece) {
							addGamePiece(new Rebuilt2026FieldObjects.FuelOnFieldSimulated(
									settledPose.getTranslation().toTranslation2d()));
						}
						continue;
					}
				}

				// Score it if in score zone
				Pair<Boolean, String> inScoreZone = gamePiece.isInScoreZone();
				if (inScoreZone.getFirst()) {
					score += gamePiece.getScoreValue(inScoreZone.getSecond());
					Logger.recordOutput("Field/Scoring/SimScore", score);
					if (gamePiece.shouldDeleteAndRemoveFromSimulation(inScoreZone.getSecond())) {
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);

					} else {
						// we should NOT delete it, instead change the game piece type.
						double magnitude = gamePiece.momentumMagnitude;
						double angle = gamePiece.momentumAngle;
						Pose3d oldPose = gamePiece.getPose3d();
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
						gamePiece = gamePiece.scoredGamePieceType(inScoreZone.getSecond(), oldPose, magnitude, angle);
						this.addGamePiece(gamePiece);
						this.competitionField.addObject(gamePiece);
					}
				}
				// YEARLYUPDATE: change the "reefscape2025fieldobjects.algaeballinfly" to the
				// year's gamepieceinfly class,
				// & add score handling to match the score zones for this year's gamepiece
				// if we're a fuel on field, if we touch the front side of the robot, we become
				// a fuel on manipulator
				if (gamePiece instanceof Rebuilt2026FieldObjects.FuelOnFieldSimulated) {
					if (isInsideMainRobotIntakeZone(gamePiece)) {
						intakeFuel(gamePiece);
					}
				}
			}
			// memory management
			gamePiecesCopy = null;
		}
		Logger.recordOutput("Field/Scoring/SimScore", score);
		competitionField.updateObjectsToDashboardAndTelemetry();
	}

	private boolean hasContact(GamePieceInSimulation gamePiece) {
		gamePiece.setTransform(GeometryConvertor.toDyn4jTransform(gamePiece.getPose3d().toPose2d()));
		Logger.recordOutput("Field/PhysicsLocation", GeometryConvertor.toWpilibPose2d(gamePiece.getTransform()));
		for (ContactConstraint<Body> contact : this.physicsWorld.getContacts(gamePiece)) {
			// Check if either body in the contact is the game piece
			if (contact.getOtherBody(gamePiece) != null) {
				// Make sure it isn't the ground
				if (contact.getOtherBody(gamePiece).getUserData() != null) {
					Object userData = contact.getOtherBody(gamePiece).getUserData();
					if (userData instanceof double[]) {
						double[] userDataArray = (double[]) userData;
						if (gamePiece.getPose3d().getZ() <= userDataArray[0]) {
							System.out.println("Contact with "
									+ contact.getOtherBody(gamePiece).getFixture(0).getShape().getClass().getName());
							return true; // Contact found
						}
					}
				}
			}
		}
		return false; // No contacts found (except with ground)
	}

	public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
		final List<Pose3d> gamePiecesPoses = new ArrayList<>();
		for (GamePieceInSimulation gamePiece : gamePieces)
			if (Objects.equals(gamePiece.getType(), type))
				gamePiecesPoses.add(gamePiece.getPose3d());

		return gamePiecesPoses;
	}

	public synchronized List<GamePieceInSimulation> getGamePiecesByType(String type) {
		final List<GamePieceInSimulation> gamePiecesPoses = new ArrayList<>();
		for (GamePieceInSimulation gamePiece : gamePieces)
			if (gamePiece.getType().equals(type))
				gamePiecesPoses.add(gamePiece);
		return gamePiecesPoses;
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * Used to simulate vision detection for algae (not implemented in 2025)
	 * 
	 * @param position current position from where we're looking
	 * @return
	 */
	private Translation3d getClosestPointOnFieldForFuel(Translation3d position) {
		double closestX = position.getX();
		double closestY = position.getY();
		if (position.getX() < FieldConstants.FUEL_DIAMETER / 2) {
			closestX = 0 + FieldConstants.FUEL_DIAMETER / 2;
		} else if (position.getX() > FieldConstants.FIELD_WIDTH
				- FieldConstants.FUEL_DIAMETER / 2) {
			closestX = FieldConstants.FIELD_WIDTH
					- FieldConstants.FUEL_DIAMETER / 2;
		}
		// now inside field, check if inside any obstacles in the field
		for (Body obstacle : physicsWorld.getBodies()) {
			if (obstacle.getFixture(0).getShape().contains(
					GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)))) {
				// if it is, move the game piece to the closest point on the obstacle
				boolean collisionDetected = obstacle.getFixture(0).getShape()
						.contains(GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)));
				if (collisionDetected) {
					double obstacleRadius = obstacle.getFixture(0).getShape().getRadius();
					Translation2d obstacleCenter = GeometryConvertor
							.toWpilibTranslation2d(obstacle.getTransform().getTranslation());
					// move the game piece that far away radially from the obstacle center, in the
					// direction of the game piece
					double angle = Math.atan2(position.getY() - obstacleCenter.getY(),
							position.getX() - obstacleCenter.getX());
					closestX = obstacleCenter.getX() + Math.cos(angle)
							+ (obstacleRadius + FieldConstants.FUEL_DIAMETER / 2);
					closestY = obstacleCenter.getY() + Math.sin(angle)
							+ (obstacleRadius + FieldConstants.FUEL_DIAMETER / 2);

				}
			}
		}
		return new Translation3d(closestX, closestY, position.getZ());
	}

	public AbstractDriveTrainSimulation getMainDriveSimulation() {
		return mainRobot;
	}

	public void addRobot(AbstractDriveTrainSimulation chassisSimulation) {
		this.physicsWorld.addBody(chassisSimulation);
		robotSimulations.add(chassisSimulation);
		this.competitionField.addObject(chassisSimulation);
	}

	private boolean isInsideMainRobotIntakeZone(GamePieceInSimulation gamePiece) {
		Pose2d robotPose = mainRobot.getPose3d().toPose2d();
		Translation2d gamePieceRobotRelative = gamePiece.getPose3d().getTranslation().toTranslation2d()
				.minus(robotPose.getTranslation())
				.rotateBy(robotPose.getRotation().unaryMinus());

		double fuelRadiusMeters = FieldConstants.FUEL_DIAMETER / 2.0;
		double minX = GeometryConstants.simIntakeFrontEdgeFromRobotCenter - fuelRadiusMeters;
		double maxX = GeometryConstants.simIntakeFrontEdgeFromRobotCenter
				+ GeometryConstants.simIntakeDepth
				+ fuelRadiusMeters;
		double maxAbsY = GeometryConstants.simIntakeWidth / 2.0 + fuelRadiusMeters;

		return gamePieceRobotRelative.getX() >= minX
				&& gamePieceRobotRelative.getX() <= maxX
				&& Math.abs(gamePieceRobotRelative.getY()) <= maxAbsY;
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	public void intakeFuel(GamePieceInSimulation gamePiece) {
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Rebuilt2026FieldObjects.FuelOnManipulator(
					Logger.getTimestamp(), GeometryConstants.intakeSpeed,
					gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	public void shootFuel() {
		GamePieceInSimulation gamePiece = getClosestFuelOnRobot();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			double speed = 0;
			// double speed = calculateObjectSpeed(mainRobot.getLinearVelocity().x,
			// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			Logger.recordOutput("Field/ShotSpeed", speed);
			// Logger.recordOutput("ShotRPM",
			// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			// TODO
			/*
			 * gamePiece = new Rebuilt2026FieldObjects.AlgaeBallInFly(
			 * TimeUtil.getLogTimeSeconds(),
			 * speed, gamePiece.getPose3d());
			 */
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}

	public void removeGamepiece(GamePieceInSimulation gamePiece) {
		this.physicsWorld.removeBody(gamePiece);
		this.competitionField.deleteObject(gamePiece);
		gamePieces.remove(gamePiece);
	}

	// Example of how to calculate the speed of an object launched by a flywheel
	/**
	 * Calculates speed of object launched by a flywheel
	 * 
	 * @apiNote THIS NEEDS STATE-SPACE!
	 * @param speed       robot speed
	 * @param flywheelRPM flywheel RPM
	 * @return speed of the object relative to the field
	 */
	static public double calculateObjectSpeed(Twist2d speed, double flywheelRPM) {
		// Formula
		// 1. Convert flywheel RPM to angular velocity in rad/s
		double angularVelocity = (flywheelRPM * 2 * Math.PI) / 60.0;
		double speedMagnitude = Math.hypot(speed.dx, speed.dy);
		return (angularVelocity * Units.inchesToMeters(2) * Units.lbsToKilograms(1))
				/ (Units.lbsToKilograms(1) + FieldConstants.Fuel.DEFAULT_MASS_KG) + speedMagnitude;
	}

	public GamePieceInSimulation getClosestGamePiece(List<Class<?>> wantedType) {
		GamePieceInSimulation closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		Set<Class<?>> allowedClasses = new HashSet<>();
		for (Class<?> gamePiece : wantedType) {
			allowedClasses.add(gamePiece);
		}

		for (GamePieceInSimulation gamePiece : gamePieces) {
			if (!allowedClasses.contains(gamePiece.getClass())) {
				continue;
			}

			double distance = gamePiece.getPose3d()
					.getTranslation()
					.getDistance(mainRobot.getPose3d().getTranslation());

			if (distance < closestDistance) {
				closestGamePiece = gamePiece;
				closestDistance = distance;
			}
		}

		return closestGamePiece;
	}

	public Pose2d getClosestGamePiecePose2d(List<Class<?>> wantedType) {
		GamePieceInSimulation closestGamePiece = getClosestGamePiece(wantedType);
		if (closestGamePiece == null) {
			return new Pose2d();
		}
		return closestGamePiece.getObjectOnFieldPose2d();
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece that is closest to the robot and is on the ground
	 */
	public GamePieceInSimulation getClosestFuelOnGround() {
		GamePieceInSimulation closestGamePiece = getClosestGamePiece(
				List.of(Rebuilt2026FieldObjects.FuelOnFieldSimulated.class));
		if (closestGamePiece == null) {
			resetField(false); // if there are no game pieces on the ground, reset the field
			return getClosestFuelOnGround(); // try again (I am aware this could be an infinite loop - G)
		}
		return closestGamePiece;
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece one that is closest to the robot and is on the robot
	 */
	public GamePieceInSimulation getClosestFuelOnRobot() {
		return getClosestGamePiece(
				List.of(Rebuilt2026FieldObjects.FuelOnManipulator.class));
	}

	public void addGamePiece(GamePieceInSimulation gamePieceInSimulation) {
		this.physicsWorld.addBody(gamePieceInSimulation);
		this.competitionField.addObject(gamePieceInSimulation);
		gamePieces.add(gamePieceInSimulation);
	}

	public CompField getCompetitionField() {
		return competitionField;
	}

	public void clearGamePieces() {
		for (GamePieceInSimulation gamePiece : gamePieces) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField
					.clearObjectsWithGivenType(gamePiece.getTypeName());
		}
		gamePieces.clear();
	}

	// YEARLYUPDATE: change these to match the year's gamepiece
	public static Pose2d getClosestGamePiece(Class<?> wantedType,
			Translation2d robotPosition) {
		GamePieceInSimulation closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		for (GamePieceInSimulation gamePiece : gamePieces) {
			Class<?> gamePieceClass = gamePiece.getClass();
			if (!(gamePieceClass.equals(wantedType))) {
				continue;
			}
			double distance = gamePiece.getPose3d().getTranslation()
					.toTranslation2d().getDistance(robotPosition);

			if (distance < closestDistance) {
				closestGamePiece = gamePiece;
				closestDistance = distance;
			}
		}
		if (closestGamePiece == null) {
			return null;
		}
		return closestGamePiece.getPose3d().toPose2d();
	}

	/**
	 * Used to simulate vision detection of other robots
	 * 
	 * @param robotPosition the current robot's pose
	 * @return
	 */
	public static Pose2d getClosestRobotPose(Translation2d robotPosition) {
		AbstractDriveTrainSimulation closestRobot = null;
		double closestDistance = Double.MAX_VALUE;
		for (AbstractDriveTrainSimulation robot : robotSimulations) {
			// if (!(robot instanceof SimplifiedHolonomicDriveSimulation)) {
			// continue;
			// }
			if (robot instanceof SwerveDriveSimulation) {
				continue;
			}
			double distance = robot.getPose3d().getTranslation().toTranslation2d()
					.getDistance(robotPosition);

			if (distance < closestDistance) {
				closestRobot = robot;
				closestDistance = distance;
			}
		}
		return closestRobot.getPose3d().toPose2d();
	}

	public void resetField(boolean preload) {
		clearGamePieces();
		placeGamePiecesOnField(preload);
		score = 0;
		scoredCoral = 0;
		scoredAlgae = 0;
		Logger.recordOutput("Field/Scoring/SimScore", score);
		Logger.recordOutput("Field/Scoring/SimCoralScoredCount", scoredCoral);
		Logger.recordOutput("Field/Scoring/SimAlgaeScoredCount", scoredAlgae);
	}

	/**
	 * place all game pieces on the field (for autonomous)
	 */
	public abstract void placeGamePiecesOnField(boolean preload);

	/**
	 * stores the obstacles on a competition field, which includes the border and
	 * the game pieces
	 */
	public static abstract class FieldObstaclesMap {
		private final List<Body> obstacles = new ArrayList<>();

		protected void addBorderLine(Translation2d startingPoint,
				Translation2d endingPoint, double obstacleHeight) {
			final Body obstacle = getObstacle(Geometry.createSegment(
					GeometryConvertor.toDyn4jVector2(startingPoint),
					GeometryConvertor.toDyn4jVector2(endingPoint)));
			obstacle.setUserData(obstacleHeight);
			obstacles.add(obstacle);
		}

		protected void addBorderLine(Translation2d startingPoint,
				Translation2d endingPoint) {
			final Body obstacle = getObstacle(Geometry.createSegment(
					GeometryConvertor.toDyn4jVector2(startingPoint),
					GeometryConvertor.toDyn4jVector2(endingPoint)));
			obstacle.setUserData(0);
			obstacles.add(obstacle);
		}

		protected void addRectangularObstacle(double width, double height,
				Pose2d pose) {
			final Body obstacle = getObstacle(
					Geometry.createRectangle(width, height));
			obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
			obstacle.setUserData(0);
			obstacles.add(obstacle);
		}

		protected void addRectangularObstacle(double width, double height,
				Pose2d pose, double obstacleHeight) {
			final Body obstacle = getObstacle(
					Geometry.createRectangle(width, height));
			obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
			obstacle.setUserData(obstacleHeight);
			obstacles.add(obstacle);
		}

		private static Body getObstacle(Convex shape) {
			final Body obstacle = new Body();
			obstacle.setMass(MassType.INFINITE);
			final BodyFixture fixture = obstacle.addFixture(shape);
			fixture.setFriction(0.8);
			fixture.setRestitution(0.6);
			return obstacle;
		}
	}
}
