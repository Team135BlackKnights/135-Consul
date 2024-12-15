package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024FieldObjects.NoteOnFieldSimulated;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePieceTag;
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
	private final static Set<AbstractDriveTrainSimulation> robotSimulations = new HashSet<>();
	private final AbstractDriveTrainSimulation mainRobot;
	private final static Set<GamePieceInSimulation> gamePieces = new HashSet<>();
	private static double score = 0;

	public CompetitionFieldSimulation(AbstractDriveTrainSimulation mainRobot,
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
			for (AbstractDriveTrainSimulation robotSimulation : robotSimulations)
				robotSimulation.simulationSubTick();
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
							.getZ() <= .03) { //collision with ground
						//make the gamepiece a ground note
						//check if the note is close enough to a speaker
						//otherwise, make it a ground note
						double momentumAngle = gamePiece.momentumAngle;
						double momentumMagnitude = gamePiece.momentumMagnitude;
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
						gamePiece = new Crescendo2024FieldObjects.NoteOnFieldSimulated(
								getClosestPointOnField(position).toTranslation2d(), momentumAngle, momentumMagnitude);
						this.addGamePiece(gamePiece);
						this.competitionField.addObject(gamePiece);
					}else if (hasContact(gamePiece) && gamePiece.isEnabled()) {
						// Flip the note velocity by Math.PI to simulate a bounce.
						//Relaunch the note from it's current position, just math.pi radians away
						double momentumMagnitude = gamePiece.momentumMagnitude * FieldConstants.EDGE_COEFFICIENT_OF_RESTITUTION;
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
						gamePiece = new Crescendo2024FieldObjects.NoteInFly(
								TimeUtil.getLogTimeSeconds(),
								momentumMagnitude, gamePiece.getPose3d().transformBy(new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,Math.PI))));
						this.addGamePiece(gamePiece);
						this.competitionField.addObject(gamePiece);
						final GamePieceInSimulation finalGamePiece = gamePiece;
						//disable the game piece collision for a short time
						new Thread(() -> {
							finalGamePiece.setEnabled(false);
							try {
								Thread.sleep(250);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
							finalGamePiece.setEnabled(true);
						}).start();
					}
				}
			}
			//memory management
			gamePiecesCopy = null;
		}
		competitionField.updateObjectsToDashboardAndTelemetry();
	}
private boolean hasContact(GamePieceInSimulation gamePiece) {
	gamePiece.setTransform(GeometryConvertor.toDyn4jTransform(gamePiece.getPose3d().toPose2d()));
	Logger.recordOutput("PhysicsLocation", GeometryConvertor.toWpilibPose2d(gamePiece.getTransform()));
	for (ContactConstraint<Body> contact : this.physicsWorld.getContacts(gamePiece)) {
		// Check if either body in the contact is the game piece
		if (contact.getOtherBody(gamePiece) != null) {
			// Make sure it isn't the ground
			if (contact.getOtherBody(gamePiece).getUserData() != null) {
				Object userData = contact.getOtherBody(gamePiece).getUserData();
				if (userData instanceof double[]) {
					double[] userDataArray = (double[]) userData;
					if (gamePiece.getPose3d().getZ() <= userDataArray[1]) {
						System.out.println("Contact with " + contact.getOtherBody(gamePiece).getFixture(0).getShape().getClass().getName());
						return true; // Contact found
					}
				}
			}
		}
	}
    return false; // No contacts found
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

	public AbstractDriveTrainSimulation getMainDriveSimulation() {
		return mainRobot;
	}

	public void addRobot(AbstractDriveTrainSimulation chassisSimulation) {
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
			double speed = 0;
			//double speed = calculateObjectSpeed(mainRobot.getLinearVelocity().x,
			// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			Logger.recordOutput("ShotSpeed", speed);
			//Logger.recordOutput("ShotRPM", (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			gamePiece = new Crescendo2024FieldObjects.NoteInFly(
					TimeUtil.getLogTimeSeconds(),
					speed, gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//Example of how to calculate the speed of an object launched by a flywheel
	@SuppressWarnings("unused")
	private double calculateObjectSpeed(double speedX, double flywheelRPM) {
		/*// 1. Convert flywheel RPM to angular velocity in rad/s
		double angularVelocity = (flywheelRPM * 2 * Math.PI) / 60.0;
		// 3. Calculate the moment of inertia for the flywheel (assuming a solid disk): I = 0.5 * m * r^2
		double momentOfInertia = 0.5 * StateSpaceConstants.Flywheel.mass * Math.pow(StateSpaceConstants.Flywheel.radius, 2);

		// 4. Calculate the kinetic energy of the flywheel: KE = 0.5 * I * ω^2
		double flywheelKineticEnergy = 0.5 * momentOfInertia * Math.pow(angularVelocity, 2);

		// 5. Assume the flywheel transfers part of its kinetic energy to the object
		// Energy transferred to the object (we'll assume full efficiency for impulse calculation)
		double energyTransferred = StateSpaceConstants.Flywheel.efficiency * flywheelKineticEnergy;

		// 6. Calculate the final velocity of the object using kinetic energy: KE = 0.5 * m * v^2 => v = sqrt(2 * KE / m)
		double velocityDueToFlywheel = Math.sqrt(2 * energyTransferred / FieldConstants.CrescendoNote.DEFAULT_MASS_KG);

		// 7. Total speed is the combination of robot speed and the flywheel-imparted speed
		//use heading of robot to determine how to use x / y components
		double totalSpeed = velocityDueToFlywheel + speedX; // x direction combines with flywheel velocity*
		return totalSpeed; // returns the total speed in m/s*/
		return 0;
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
			if (!(gamePiece instanceof NoteOnFieldSimulated)) {
				continue;
			}
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
		AbstractDriveTrainSimulation closestRobot = null;
		double closestDistance = Double.MAX_VALUE;
		for (AbstractDriveTrainSimulation robot : robotSimulations) {
			//if (!(robot instanceof SimplifiedHolonomicDriveSimulation)) {
			//	continue;
			//}
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
				Translation2d endingPoint, double[] obstacleData) {
			final Body obstacle = getObstacle(Geometry.createSegment(
					GeometryConvertor.toDyn4jVector2(startingPoint),
					GeometryConvertor.toDyn4jVector2(endingPoint)));
					obstacle.setUserData(obstacleData);
			obstacles.add(obstacle);
		}

		protected void addRectangularObstacle(double width, double height,
				Pose2d pose, double[] obstacleData) {
			final Body obstacle = getObstacle(
					Geometry.createRectangle(width, height));
			obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(pose));
			obstacle.setUserData(obstacleData);
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
