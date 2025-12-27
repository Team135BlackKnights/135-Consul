package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.GeometryConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects.AlgaeBallOnFieldStatic;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedBattery;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.ReefHeight;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.ReefscapeCoral;
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
	private final AbstractDriveTrainSimulation mainRobot;
	private final static Set<GamePieceInSimulation> gamePieces = new HashSet<>();
	private static double score = 0, scoredCoral = 0, scoredAlgae = 0;

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

	public void addPoints(int points) {
		score += points;
		Logger.recordOutput("Scoring/SimScore", score);
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
			// go through all game pieces
			Set<GamePieceInSimulation> gamePiecesCopy = new HashSet<>(gamePieces); // Create a copy of the gamePieces
																					// set
			for (GamePieceInSimulation gamePiece : gamePiecesCopy) { // Iterate over the copy
				//Score it if in score zone
				Pair<Boolean, String> inScoreZone = gamePiece.isInScoreZone();
				if (inScoreZone.getFirst()) {
					score += gamePiece.getScoreValue(inScoreZone.getSecond());
					Logger.recordOutput("Scoring/SimScore", score);
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
				//YEARLYUPDATE: change the "reefscape2025fieldobjects.algaeballinfly" to the year's gamepieceinfly class, 
				// & add score handling to match the score zones for this year's gamepiece 
				if (gamePiece instanceof Reefscape2025FieldObjects.AlgaeBallInFly) {
					Translation3d position = gamePiece.getPose3d().getTranslation();
					if (gamePiece.getPose3d().getTranslation()
							.getZ() < 2) { // check for collision with Net


						if (hasContact(gamePiece) && gamePiece.isEnabled()) { // bounce off obstacles
							// Flip the note velocity by Math.PI to simulate a bounce.
							// Relaunch the note from it's current position, just math.pi radians away
							double momentumMagnitude = gamePiece.momentumMagnitude *
									FieldConstants.AlgaeBall.EDGE_COEFFICIENT_OF_RESTITUTION;
							this.physicsWorld.removeBody(gamePiece);
							this.competitionField.deleteObject(gamePiece);
							gamePieces.remove(gamePiece);
							gamePiece = new Reefscape2025FieldObjects.AlgaeBallInFly(
									TimeUtil.getLogTimeSeconds(),
									momentumMagnitude, gamePiece.getPose3d().transformBy(
											new Transform3d(new Translation3d(0, 0, 0),
													new Rotation3d(0, 0, Math.PI))));
							this.addGamePiece(gamePiece);
							this.competitionField.addObject(gamePiece);
							final GamePieceInSimulation finalGamePiece = gamePiece;
							// disable the game piece collision for a short time
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
						double momentumAngle = gamePiece.momentumAngle;
						double momentumMagnitude = gamePiece.momentumMagnitude;
						this.physicsWorld.removeBody(gamePiece);
						this.competitionField.deleteObject(gamePiece);
						gamePieces.remove(gamePiece);
						gamePiece = new Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated(
								getClosestPointOnFieldForAlgae(position).toTranslation2d(),
								momentumAngle,
								momentumMagnitude);
						this.addGamePiece(gamePiece);
						this.competitionField.addObject(gamePiece);
					} else if (gamePiece.getPose3d().getTranslation()
							.getZ() > 2) {
						Pose3d pose = gamePiece.getPose3d();
						if (pose.getX() > 8.1 && pose.getX() < 9.35) {
							// in the net
							score += 4;
							scoredAlgae++;
							Logger.recordOutput("Scoring/SimAlgaeScoredCount", scoredAlgae);
							Logger.recordOutput("Scoring/SimScore", score);
							this.physicsWorld.removeBody(gamePiece);
							this.competitionField.deleteObject(gamePiece);
							gamePieces.remove(gamePiece);
							// put a static algae where it was
							gamePiece = new AlgaeBallOnFieldStatic(pose);
							this.addGamePiece(gamePiece);
							this.competitionField.addObject(gamePiece);

						}
					}
				}

			}
			// memory management
			gamePiecesCopy = null;
		}
		Logger.recordOutput("Scoring/SimScore", score);
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

	//YEARLYUPDATE: change these to match the year's gamepiece
	/**Used to simulate vision detection for algae (not implemented in 2025)
	 * 
	 * @param position current position from where we're looking
	 * @return
	 */
	private Translation3d getClosestPointOnFieldForAlgae(Translation3d position) {
		double closestX = position.getX();
		double closestY = position.getY();
		if (position.getX() < FieldConstants.ALGAE_BALL_DIAMETER / 2) {
			closestX = 0 + FieldConstants.ALGAE_BALL_DIAMETER / 2;
		} else if (position.getX() > FieldConstants.FIELD_WIDTH
				- FieldConstants.ALGAE_BALL_DIAMETER / 2) {
			closestX = FieldConstants.FIELD_WIDTH
					- FieldConstants.ALGAE_BALL_DIAMETER / 2;
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
							+ (obstacleRadius + FieldConstants.ALGAE_BALL_DIAMETER / 2);
					closestY = obstacleCenter.getY() + Math.sin(angle)
							+ (obstacleRadius + FieldConstants.ALGAE_BALL_DIAMETER / 2);

				}
			}
		}
		return new Translation3d(closestX, closestY, position.getZ());
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	//** Used to simulate vision detection for coral (not implemented in 2025)*/
	private Translation3d getClosestPointOnFieldForCoral(Translation3d position) {
		double closestX = position.getX();
		double closestY = position.getY();
		if (position.getX() < FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2) {
			closestX = 0 + FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2;
		} else if (position.getX() > FieldConstants.FIELD_WIDTH
				- FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2) {
			closestX = FieldConstants.FIELD_WIDTH
					- FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2;
		}
		if (position.getY() <= FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2) {
			closestY = 0 + FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2;
		} else if (position.getY() > FieldConstants.FIELD_HEIGHT
				- FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2) {
			closestY = FieldConstants.FIELD_HEIGHT
					- FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2;
		}
		// now inside field, check if inside any obstacles in the field
		for (Body obstacle : physicsWorld.getBodies()) {
			if (obstacle.getFixture(0).getShape().contains(
					GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)))) {
				// if it is, move the note to the closest point on the obstacle
				boolean collisionDetected = obstacle.getFixture(0).getShape()
						.contains(GeometryConvertor.toDyn4jVector2(new Translation2d(closestX, closestY)));
				if (collisionDetected) {
					double obstacleRadius = obstacle.getFixture(0).getShape().getRadius();
					Translation2d obstacleCenter = GeometryConvertor
							.toWpilibTranslation2d(obstacle.getTransform().getTranslation());
					// move the note that far away radially from the obstacle center, in the
					// direction of the note
					double angle = Math.atan2(position.getY() - obstacleCenter.getY(),
							position.getX() - obstacleCenter.getX());
					closestX = obstacleCenter.getX() + Math.cos(angle)
							+ (obstacleRadius + FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2);
					closestY = obstacleCenter.getY() + Math.sin(angle)
							+ (obstacleRadius + FieldConstants.REEFSCAPE_CORAL_DIAMETER / 2);

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
	//YEARLYUPDATE: change these to match the year's gamepiece 
	public void intakeAlgae() {
		GamePieceInSimulation gamePiece = getClosestAlgaeOnGround();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Reefscape2025FieldObjects.AlgaeBallOnManipulator(
					Logger.getTimestamp(), GeometryConstants.intakeSpeed,
					gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void shootAlgae() {
		GamePieceInSimulation gamePiece = getClosestAlgaeOnRobot();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			double speed = 0;
			// double speed = calculateObjectSpeed(mainRobot.getLinearVelocity().x,
			// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			Logger.recordOutput("ShotSpeed", speed);
			// Logger.recordOutput("ShotRPM",
			// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
			gamePiece = new Reefscape2025FieldObjects.AlgaeBallInFly(
					TimeUtil.getLogTimeSeconds(),
					speed, gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void shootAlgae(Pose3d startPose) {

		double speed = 1;
		// double speed = calculateObjectSpeed(mainRobot.getLinearVelocity().x,
		// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
		// Logger.recordOutput("ShotRPM",
		// (RobotContainer.flywheelS.getTopRPM()+RobotContainer.flywheelS.getBottomRPM())/2);
		GamePieceInSimulation gamePiece = new Reefscape2025FieldObjects.AlgaeBallInFly(
				TimeUtil.getLogTimeSeconds(),
				speed, startPose);
		this.addGamePiece(gamePiece);
		this.competitionField.addObject(gamePiece);

	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void intakeCoral() {
		GamePieceInSimulation gamePiece = getClosestCoralOnGround();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Reefscape2025FieldObjects.ReefscapeCoralOnManipulator();
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void shootCoral() {
		GamePieceInSimulation gamePiece = getClosestCoralOnRobot();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Reefscape2025FieldObjects.ReefscapeCoralInFly(
					TimeUtil.getLogTimeSeconds(), gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void intakeNearestGamePiece() {
		GamePieceInSimulation gamePiece = getClosestGamePieceOnGround();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Reefscape2025FieldObjects.ReefscapeCoralOnManipulator();
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	public void shootNearestGamePiece() {
		GamePieceInSimulation gamePiece = getClosestGamePieceOnRobot();
		if (gamePiece != null) {
			this.physicsWorld.removeBody(gamePiece);
			this.competitionField.deleteObject(gamePiece);
			gamePieces.remove(gamePiece);
			gamePiece = new Reefscape2025FieldObjects.ReefscapeCoralInFly(
					TimeUtil.getLogTimeSeconds(), gamePiece.getPose3d());
			this.addGamePiece(gamePiece);
			this.competitionField.addObject(gamePiece);
		}
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
				/ (Units.lbsToKilograms(1) + FieldConstants.ReefscapeCoral.DEFAULT_MASS_KG) + speedMagnitude;
	}

	private GamePieceInSimulation getClosestGamePiece(List<Class<?>> wantedType) {
		GamePieceInSimulation closestGamePiece = null;
		double closestDistance = Double.MAX_VALUE;
		Set<Class<?>> allowedClasses = new HashSet<>();
		for (Class<?> gamePiece : wantedType) {
			allowedClasses.add(gamePiece.getClass());
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
	//YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece that is closest to the robot and is on the ground
	 */
	public GamePieceInSimulation getClosestAlgaeOnGround() {
		GamePieceInSimulation closestGamePiece = getClosestGamePiece(
				List.of(Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated.class));
		if (closestGamePiece == null) {
			resetField(false); // if there are no game pieces on the ground, reset the field
			return getClosestAlgaeOnGround(); // try again (I am aware this could be an infinite loop - G)
		}
		return closestGamePiece;
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece that is closest to the robot and is on the ground
	 */
	public GamePieceInSimulation getClosestCoralOnGround() {
		GamePieceInSimulation closestGamePiece = getClosestGamePiece(
				List.of(Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated.class));
		if (closestGamePiece == null) {
			resetField(false); // if there are no game pieces on the ground, reset the field
			return getClosestCoralOnGround(); // try again (I am aware this could be an infinite loop - G)
		}
		return closestGamePiece;
	}

	public GamePieceInSimulation getClosestGamePieceOnGround() {
		return getClosestGamePiece(List.of(
				Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated.class,
				Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated.class));
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece one that is closest to the robot and is on the robot
	 */
	public GamePieceInSimulation getClosestAlgaeOnRobot() {
		return getClosestGamePiece(
				List.of(Reefscape2025FieldObjects.AlgaeBallOnManipulator.class));
	}
	//YEARLYUPDATE: change these to match the year's gamepiece
	/**
	 * @return the game piece two that is closest to the robot and is on the robot
	 */
	public GamePieceInSimulation getClosestCoralOnRobot() {
		return getClosestGamePiece(
				List.of(Reefscape2025FieldObjects.ReefscapeCoralOnManipulator.class));
	}

	/**
	 * @return the game piece that is closest to the robot and is on the robot
	 */
	public GamePieceInSimulation getClosestGamePieceOnRobot() {
		return getClosestGamePiece(List.of(
				Reefscape2025FieldObjects.AlgaeBallOnManipulator.class,
				Reefscape2025FieldObjects.ReefscapeCoralOnManipulator.class));
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
	//YEARLYUPDATE: change these to match the year's gamepiece
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
		Logger.recordOutput("Scoring/SimScore", score);
		Logger.recordOutput("Scoring/SimCoralScoredCount", scoredCoral);
		Logger.recordOutput("Scoring/SimAlgaeScoredCount", scoredAlgae);
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
