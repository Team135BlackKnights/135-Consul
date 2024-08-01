package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
	private final Set<HolonomicChassisSimulation> robotSimulations = new HashSet<>();
	private final HolonomicChassisSimulation mainRobot;
	private final Set<GamePieceInSimulation> gamePieces;

	public CompetitionFieldSimulation(HolonomicChassisSimulation mainRobot,
			FieldObstaclesMap obstaclesMap) {
		this.competitionField = new CompField(mainRobot);
		this.mainRobot = mainRobot;
		this.physicsWorld = new World<>();
		this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
		for (Body obstacle : obstaclesMap.obstacles)
			this.physicsWorld.addBody(obstacle);
		this.gamePieces = new HashSet<>();
		this.physicsWorld.addBody(mainRobot);
		this.robotSimulations.add(mainRobot);
	}

	public void updateSimulationWorld() {
		final double subPeriodSeconds = Robot.defaultPeriodSecs
				/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
		// move through 5 sub-periods in each update
		for (int i = 0; i < DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD; i++) {
			this.physicsWorld.step(1, subPeriodSeconds);
			for (HolonomicChassisSimulation robotSimulation : robotSimulations)
				robotSimulation.updateSimulationSubPeriod(i, subPeriodSeconds);
		}
		competitionField.updateObjectsToDashboardAndTelemetry();
	}

	public SwerveDriveSimulation getSwerveDriveSimulation() {
		return (SwerveDriveSimulation) mainRobot;
	}

	public void addRobot(HolonomicChassisSimulation chassisSimulation) {
		this.physicsWorld.addBody(chassisSimulation);
		this.robotSimulations.add(chassisSimulation);
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
					Logger.getTimestamp(), Constants.GeometryConstants.shotSpeed,
					gamePiece.getPose3d(), Robot.isRed ? FieldConstants.RED_SPEAKER
							: FieldConstants.BLUE_SPEAKER);
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
		this.gamePieces.add(gamePieceInSimulation);
	}

	public CompField getCompetitionField() { return competitionField; }

	public void clearGamePieces() {
		for (GamePieceInSimulation gamePiece : this.gamePieces) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField
					.clearObjectsWithGivenType(gamePiece.getTypeName());
		}
		this.gamePieces.clear();
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
