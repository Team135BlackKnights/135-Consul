package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePieceTag;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.CompetitionFieldUtils.CompField;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.maths.GeometryConvertor;
import frc.robot.utils.maths.TimeUtil;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * this class simulates the physical behavior of all the objects on field should
 * only be created during a robot simulation (not in real or replay mode)
 */
public abstract class CompetitionFieldSimulation {
	private final World<Body> physicsWorld;
	private final CompField competitionField;
	private final static Set<HolonomicChassisSimulation> robotSimulations = new HashSet<>();
	private final HolonomicChassisSimulation mainRobot;
	private final static Set<GamePieceInSimulation> gamePieces = new HashSet<>();
	private static double score = 0;

	public CompetitionFieldSimulation(HolonomicChassisSimulation mainRobot,
			FieldObstaclesMap obstaclesMap) {
		this.competitionField = new CompField(mainRobot);
		this.mainRobot = mainRobot;
		this.physicsWorld = new World<>();
		this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
		for (Body obstacle : obstaclesMap.obstacles)
			this.physicsWorld.addBody(obstacle);
		this.physicsWorld.addBody(mainRobot);
		robotSimulations.add(mainRobot);
	}

	public void updateSimulationWorld() {
		final double subPeriodSeconds = Robot.defaultPeriodSecs
				/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
		// move through 5 sub-periods in each update
		for (int i = 0; i < DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD; i++) {
			this.physicsWorld.step(1, subPeriodSeconds);
			for (HolonomicChassisSimulation robotSimulation : robotSimulations)
				robotSimulation.updateSimulationSubPeriod(i, subPeriodSeconds);
			//go through all game pieces
			Set<GamePieceInSimulation> gamePiecesCopy = new HashSet<>(gamePieces); // Create a copy of the gamePieces set
			for (GamePieceInSimulation gamePiece : gamePiecesCopy) { // Iterate over the copy
				//if gamepiece is an air note, check if we've hit the ground
				if (gamePiece.getTag() == GamePieceTag.IN_AIR) {
					Translation3d position = getClosestPointOnField(
							gamePiece.getPose3d().getTranslation());
					//check if the note is close enough to a speaker
					if (isCloseToSpeaker(position)) {
						//if it is, make it a speaker note
						score += FieldConstants.SPEAKER_SCORE;
						Logger.recordOutput("SimScore", score);
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
					} else if (gamePiece.getPose3d().getTranslation()
							.getZ() <= FieldConstants.NOTE_HEIGHT) { //collision with ground
						//make the gamepiece a ground note
						//check if the note is close enough to a speaker
						//otherwise, make it a ground note
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
						gamePiece = new Crescendo2024FieldObjects.NoteOnFieldSimulated(
								position.toTranslation2d());
						this.addGamePiece(gamePiece);
						this.competitionField.addObject(gamePiece);
					}
				}
			}
			//memory management
			gamePiecesCopy = null;
		}
		competitionField.updateObjectsToDashboardAndTelemetry();
	}

	private boolean isCloseToSpeaker(Translation3d position) {
		//check if the note is close enough to a speaker
		if (position.getDistance(
				FieldConstants.BLUE_SPEAKER) <= FieldConstants.SPEAKER_COLLISION_RADIUS) {
			return true;
		}
		if (position.getDistance(
				FieldConstants.RED_SPEAKER) <= FieldConstants.SPEAKER_COLLISION_RADIUS) {
			return true;
		}
		return false;
	}

	private Translation3d getClosestPointOnField(Translation3d position) {
		double closestX = position.getX();
		double closestY = position.getY();
		if (position.getX() < FieldConstants.NOTE_DIAMETER / 2) {
			closestX = 0 + FieldConstants.NOTE_DIAMETER / 2;
		} else if (position.getX() > FieldConstants.FIELD_WIDTH
				- FieldConstants.NOTE_DIAMETER / 2) {
			closestX = FieldConstants.FIELD_WIDTH
					- FieldConstants.NOTE_DIAMETER / 2;
		}
		if (position.getY() <= FieldConstants.NOTE_DIAMETER / 2) {
			closestY = 0 + FieldConstants.NOTE_DIAMETER / 2;
		} else if (position.getY() > FieldConstants.FIELD_HEIGHT
				- FieldConstants.NOTE_DIAMETER / 2) {
			closestY = FieldConstants.FIELD_HEIGHT
					- FieldConstants.NOTE_DIAMETER / 2;
		}
		//now inside field, check if inside any obstacles in the field
		for (Body obstacle : physicsWorld.getBodies()) {
			if (obstacle.getFixture(0).getShape().contains(
					GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)))) {
				//if it is, move the note to the closest point on the obstacle
				boolean collisionDetected = obstacle.getFixture(0).getShape().contains(GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)));
				if (collisionDetected) {
					double obstacleRadius = obstacle.getFixture(0).getShape().getRadius();
					Translation2d obstacleCenter = GeometryConvertor.toWpilibTranslation2d(obstacle.getTransform().getTranslation());
					//move the note that far away radially from the obstacle center, in the direction of the note
					double angle = Math.atan2(position.getY() - obstacleCenter.getY(), position.getX() - obstacleCenter.getX());
					closestX = obstacleCenter.getX() + Math.cos(angle) + (obstacleRadius + FieldConstants.NOTE_DIAMETER / 2);
					closestY = obstacleCenter.getY() + Math.sin(angle) + (obstacleRadius + FieldConstants.NOTE_DIAMETER / 2);
					
				}
			}
		}
		return new Translation3d(closestX, closestY, position.getZ());
	}

	public HolonomicChassisSimulation getMainDriveSimulation() {
		return mainRobot;
	}

	public void addRobot(HolonomicChassisSimulation chassisSimulation) {
		this.physicsWorld.addBody(chassisSimulation);
		robotSimulations.add(chassisSimulation);
		this.competitionField.addObject(chassisSimulation);
	}

	public void intakeNote() {
		GamePieceInSimulation gamePiece = getClosestGamePieceOnGround();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Crescendo2024FieldObjects.NoteOnManipulator(
					Logger.getTimestamp(), GeometryConstants.intakeSpeed,
					gamePiece.getPose3d(), GeometryConstants.launcherTransform);
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}

	public void shootNote() {
		GamePieceInSimulation gamePiece = getClosestGamePieceOnRobot();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Crescendo2024FieldObjects.NoteInFly(
					TimeUtil.getLogTimeSeconds(),
					Constants.GeometryConstants.shotSpeed, gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}

	private GamePieceInSimulation getClosestGamePiece(GamePieceTag tag) {
		GamePieceInSimulation closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		for (GamePieceInSimulation gamePiece : gamePieces) {
			if (gamePiece.getTag() != tag)
				continue;
			double distance = gamePiece.getPose3d().getTranslation()
					.getDistance(mainRobot.getPose3d().getTranslation());
			if (distance < closestDistance) {
				closestGamePiece = gamePiece;
				closestDistance = distance;
			}
		}
		return closestGamePiece;
	}

	/**
	 * @return the game piece that is closest to the robot and is on the ground
	 */
	public GamePieceInSimulation getClosestGamePieceOnGround() {
		GamePieceInSimulation closestGamePiece = getClosestGamePiece(
				GamePieceTag.ON_GROUND);
		if (closestGamePiece == null) {
			resetField(false); // if there are no game pieces on the ground, reset the field
			return getClosestGamePieceOnGround(); // try again   (I am aware this could be an infinite loop - G)
		}
		return closestGamePiece;
	}

	/**
	 * @return the game piece that is closest to the robot and is on the robot
	 */
	public GamePieceInSimulation getClosestGamePieceOnRobot() {
		return getClosestGamePiece(GamePieceTag.IN_ROBOT);
	}

	public void addGamePiece(GamePieceInSimulation gamePieceInSimulation) {
		this.physicsWorld.addBody(gamePieceInSimulation);
		this.competitionField.addObject(gamePieceInSimulation);
		gamePieces.add(gamePieceInSimulation);
	}

	public CompField getCompetitionField() { return competitionField; }

	public void clearGamePieces() {
		for (GamePieceInSimulation gamePiece : gamePieces) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField
					.clearObjectsWithGivenType(gamePiece.getTypeName());
		}
		gamePieces.clear();
	}

	public static Translation2d getClosestGamePiece(
			Translation2d robotPosition) {
		GamePieceInSimulation closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		for (GamePieceInSimulation gamePiece : gamePieces) {
			double distance = gamePiece.getPose3d().getTranslation()
					.toTranslation2d().getDistance(robotPosition);
			if (distance < closestDistance) {
				closestGamePiece = gamePiece;
				closestDistance = distance;
			}
		}
		return closestGamePiece.getPose3d().getTranslation().toTranslation2d();
	}

	public static Pose2d getClosestRobotPose(Translation2d robotPosition) {
		HolonomicChassisSimulation closestRobot = null;
		double closestDistance = Double.MAX_VALUE;
		for (HolonomicChassisSimulation robot : robotSimulations) {
			if (!(robot instanceof OpponentRobotSimulation)) {
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
				Translation2d endingPoint) {
			final Body obstacle = getObstacle(Geometry.createSegment(
					GeometryConvertor.toDyn4jVector2(startingPoint),
					GeometryConvertor.toDyn4jVector2(endingPoint)));
			obstacles.add(obstacle);
		}

		protected void addRectangularObstacle(double width, double height,
				Pose2d pose) {
			final Body obstacle = getObstacle(
					Geometry.createRectangle(width, height));
			obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
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
