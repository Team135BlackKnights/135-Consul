// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.OrchestraC;
import frc.robot.commands.StaticCharacterization;
import frc.robot.commands.auto.BranchAutoSegment;
import frc.robot.commands.drive.DriveToTargetUsingDriveAndAimAtPose;
import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import frc.robot.subsystems.drive.Mecanum.MecanumIO;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSparkBase;
import frc.robot.subsystems.drive.Mecanum.MecanumIOTalonFX;
import frc.robot.subsystems.drive.FastSwerve.ModuleIO;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOCShifting;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOKrakenFOCWithThrifty;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIOTalonFX;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.AIRobotInSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.MecanumDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.TankDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.GyroSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.utils.vision.VisionConstants;

import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;
import frc.robot.utils.drive.Sensors.GyroIOSim;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.DriverStationHID;
/**
 * This code depends on WPILib 2025, Choreo 2025, PhotonLib 2025, Studica,
 * Phoenix-6 2025 (non-replay), REVLib 2025, URCL, GrappleLib 2025, AKit 2025,
 * and PathplannerLib 2025.
 * IT WILL NOT WORK WITHOUT ANY OF THESE!
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	public static Vision visionS;
	private final LoggedDashboardChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static DriverStationHID dsHIDHandler = new DriverStationHID(2);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			yButtonDrive = new JoystickButton(driveController, 4), // used for Aim/Drive to pose
			bButtonDrive = new JoystickButton(driveController, 2),
			aButtonDrive = new JoystickButton(driveController, 1),
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	public static int currentTest = 0;
	public static String piConnection = "DISCONNECTED";
	@AutoLogOutput(key = "RobotState/currentPath")
	public static String currentPath = "";
	public static Field2d field = new Field2d();
	public static double[] knownInputs = new double[2]; // number of inputs to AI
	public static double[] knownOutputs = new double[2]; // number of outputs to AI
	public static List<Double> currentAiOutputs = new ArrayList<Double>(4); // total values for AI

	public enum GamePieceState {
		NO_GAME_PIECE, HAS_NOTE, ABORT
	}
	public static Translation2d[] gamePieceLocations = FieldConstants.NOTE_INITIAL_POSITIONS;

	@AutoLogOutput(key = "RobotState/currentGamePieceStatus")
	public static GamePieceState currentGamePieceStatus = GamePieceState.NO_GAME_PIECE;
	public static boolean userDrive = true;
	// Simulation
	public static Crescendo2024FieldSimulation fieldSimulation = null;
	public static Command currentAuto;

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands. y * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		Logger.recordOutput("DSHID/DSHIDLedPattern", dsHIDHandler.getCurrentLEDPattern().toString());
		// We check to see what drivetrain type we have here, and create the correct
		// drivetrain system based on that.
		// If we get something wacky, throw an error
		List<Pair<String, Command>> autoCommands = new ArrayList<>();
		switch (Constants.currentMode) {
			case REAL:
				switch (DriveConstants.driveType) {
					case SWERVE:

						switch (DriveConstants.robotMotorController) {
							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:
								switch (DriveConstants.gyroType) {
									case NAVX:
										switch (DriveConstants.swerveModuleType) {
											case SHIFTING_THIFTYSWERVE:
												// ignore encoder type, assume cancoder
												drivetrainS = new Swerve(new GyroIONavX(),
														new ModuleIOKrakenFOCShifting(0),
														new ModuleIOKrakenFOCShifting(1),
														new ModuleIOKrakenFOCShifting(2),
														new ModuleIOKrakenFOCShifting(3));
												break;
											case THRIFTYSWERVE:
											case SDSMK4I:
												if (DriveConstants.useThriftyEncoder) {
													drivetrainS = new Swerve(new GyroIONavX(),
															new ModuleIOKrakenFOCWithThrifty(0),
															new ModuleIOKrakenFOCWithThrifty(1),
															new ModuleIOKrakenFOCWithThrifty(2),
															new ModuleIOKrakenFOCWithThrifty(3));
												} else {
													drivetrainS = new Swerve(new GyroIONavX(),
															new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
															new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
												}
												break;
										}
										break;
									case PIGEON:
										switch (DriveConstants.swerveModuleType) {
											case SHIFTING_THIFTYSWERVE:
												drivetrainS = new Swerve(new GyroIOPigeon2(),
														new ModuleIOKrakenFOCShifting(0),
														new ModuleIOKrakenFOCShifting(1),
														new ModuleIOKrakenFOCShifting(2),
														new ModuleIOKrakenFOCShifting(3));
												break;
											case THRIFTYSWERVE:
											case SDSMK4I:
												drivetrainS = new Swerve(new GyroIOPigeon2(),
														new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
														new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
												break;
										}
										break;
									default:
										break;
								}
								break;
							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:
								switch (DriveConstants.gyroType) {
									case NAVX:
										drivetrainS = new Swerve(new GyroIONavX(),
												new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
												new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
										break;
									case PIGEON:
										drivetrainS = new Swerve(new GyroIOPigeon2(),
												new ModuleIOSparkBase(0), new ModuleIOSparkBase(1),
												new ModuleIOSparkBase(2), new ModuleIOSparkBase(3));
									default:
										break;
								}
								break;
						}
						break;
					case TANK:
						switch (DriveConstants.robotMotorController) {
							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Tank(
												new TankIOTalonFX(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Tank(new TankIOTalonFX(new GyroIONavX()));
										break;
								}
								break;
							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Tank(
												new TankIOSparkBase(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Tank(new TankIOSparkBase(new GyroIONavX()));
										break;
								}
								break;
						}
						break;
					case MECANUM:
						switch (DriveConstants.robotMotorController) {
							case CTRE_ON_RIO:
							case CTRE_ON_CANIVORE:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Mecanum(
												new MecanumIOTalonFX(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Mecanum(
												new MecanumIOTalonFX(new GyroIONavX()));
										break;
								}
								break;
							case NEO_SPARK_MAX:
							case VORTEX_SPARK_FLEX:
								switch (DriveConstants.gyroType) {
									case PIGEON:
										drivetrainS = new Mecanum(
												new MecanumIOSparkBase(new GyroIOPigeon2()));
										break;
									case NAVX:
										drivetrainS = new Mecanum(
												new MecanumIOSparkBase(new GyroIONavX()));
										break;
								}
								break;
						}
						break;
					// Placeholder values
					default:
						throw new IllegalArgumentException(
								"Unknown drivetrain implementation type, please check DriveConstants.java!");
				}
				visionS = new Vision(
						new VisionIOPhotonVision(VisionConstants.FLCamName, VisionConstants.robotToFL),
						new VisionIOPhotonVision(VisionConstants.FRCamName, VisionConstants.robotToFR),
						new VisionIOPhotonVision(VisionConstants.BLCamName, VisionConstants.robotToBL),
						new VisionIOPhotonVision(VisionConstants.BRCamName, VisionConstants.robotToBR));
				autoCommands.addAll(Arrays.asList(
				// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
				// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));
				break;
			case SIM:
				GyroSimulation gyroSimulation = null;
				switch (DriveConstants.gyroType) {
					case PIGEON:
						gyroSimulation = GyroSimulation.createPigeon2();
						break;
					case NAVX:
						gyroSimulation = GyroSimulation.createNav2X();
						break;
				}
				switch (DriveConstants.driveType) {
					case SWERVE:
						SwerveModuleSimulation[] moduleSimulations = new SwerveModuleSimulation[4];
						ModuleIO[] moduleIOSims = new ModuleIO[4];
						for (int i = 0; i < 4; i++) {
							switch (DriveConstants.swerveModuleType) {
								case SDSMK4I:
									moduleSimulations[i] = SwerveModuleSimulation
											.getMark4i(DriveConstants.getDriveTrainMotors(1),
													DriveConstants.getDriveTrainMotors(1),
													DriveConstants.gripType.cof, 2)
											.get();
									break;
								case THRIFTYSWERVE:
								case SHIFTING_THIFTYSWERVE:
									moduleSimulations[i] = SwerveModuleSimulation
											.getThriftySwerve(DriveConstants.getDriveTrainMotors(1),
													DriveConstants.getDriveTrainMotors(1),
													DriveConstants.gripType.cof, 2)
											.get();
									break;
								default:
									throw new IllegalArgumentException(
											"Unknown implementation type for module, please check DriveConstants.java!");
							}
							moduleIOSims[i] = new ModuleIOSim(moduleSimulations[i]);
						}

						drivetrainS = new Swerve(new GyroIOSim(gyroSimulation), moduleIOSims[0],
								moduleIOSims[1], moduleIOSims[2], moduleIOSims[3]);
						SwerveDriveSimulation driveSim = new SwerveDriveSimulation(
								DriveConstants.mainRobotProfile.robotMass,
								DriveConstants.kBumperToBumperWidth, DriveConstants.kBumperToBumperLength,
								new SwerveModuleSimulation[] { moduleSimulations[0], moduleSimulations[1],
										moduleSimulations[2], moduleSimulations[3]
								}, DriveConstants.kModuleTranslations, gyroSimulation,
								FieldConstants.START_POSE, drivetrainS::resetPose);
						fieldSimulation = new Crescendo2024FieldSimulation(driveSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...
						break;
					case TANK:
						final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(
								DriveConstants.kChassisWidth);
						final GyroIOSim tankGyroIOSim = new GyroIOSim(gyroSimulation);
						TankIOSim tankIOSim = new TankIOSim(tankGyroIOSim);
						drivetrainS = new Tank(tankIOSim);
						TankDriveSimulation tankSim = new TankDriveSimulation(DriveConstants.mainRobotProfile,
								gyroSimulation,
								diffKinematics,
								FieldConstants.START_POSE,
								(Tank) drivetrainS,
								tankIOSim,
								drivetrainS::resetPose);
						fieldSimulation = new Crescendo2024FieldSimulation(tankSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...

						break;
					default:
						final MecanumDriveKinematics mechKinematics = new MecanumDriveKinematics(
								DriveConstants.kModuleTranslations[0],
								DriveConstants.kModuleTranslations[1],
								DriveConstants.kModuleTranslations[2],
								DriveConstants.kModuleTranslations[3]);
						final GyroIOSim mecanumGyroIOSim = new GyroIOSim(gyroSimulation);
						MecanumIOSim mecanumIOSim = new MecanumIOSim(mecanumGyroIOSim);
						drivetrainS = new Mecanum(mecanumIOSim);
						MecanumDriveSimulation mecanumSim = new MecanumDriveSimulation(DriveConstants.mainRobotProfile,
								gyroSimulation,
								mechKinematics,
								FieldConstants.START_POSE,
								(Mecanum) drivetrainS,
								mecanumIOSim,
								drivetrainS::resetPose);
						fieldSimulation = new Crescendo2024FieldSimulation(mecanumSim);
						fieldSimulation.placeGamePiecesOnField(true);
						AIRobotInSimulation.startOpponentRobotSimulations(); // Start your engines...
						break;
				}
				autoCommands.addAll(Arrays.asList(
						// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
						// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
						new Pair<String, Command>("SmartShoot", new PrintCommand("SmartShoot")),
						new Pair<String, Command>("SmartIntake", Commands.none()),
						new Pair<String, Command>("BranchIntakeToSpike3", new BranchAutoSegment(drivetrainS, new Pose2d(FieldConstants.NOTE_INITIAL_POSITIONS[0], new Rotation2d()), 1, 1, true)),
						new Pair<String, Command>("BranchIntakeToSpike1", new BranchAutoSegment(drivetrainS, new Pose2d(FieldConstants.NOTE_INITIAL_POSITIONS[2], new Rotation2d()), 0,  1, true))
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));
				visionS = new Vision(
						new VisionIOPhotonVisionSim(VisionConstants.FLCamName, VisionConstants.robotToFL, () -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(VisionConstants.FRCamName, VisionConstants.robotToFR, () -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(VisionConstants.BLCamName, VisionConstants.robotToBL, () -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()),
						new VisionIOPhotonVisionSim(VisionConstants.BRCamName, VisionConstants.robotToBR, () -> fieldSimulation.getMainDriveSimulation().getPose3d().toPose2d()));
				break;
			default:
				switch (DriveConstants.driveType) {
					case SWERVE:
						drivetrainS = new Swerve(new GyroIO() {
						}, new ModuleIO() {
						},
								new ModuleIO() {
								}, new ModuleIO() {
								}, new ModuleIO() {
								});
						break;
					case TANK:
						drivetrainS = new Tank(new TankIO() {
						});
						break;
					case MECANUM:
						drivetrainS = new Mecanum(new MecanumIO() {
						});
				}
				autoCommands.addAll(Arrays.asList(
				// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
				// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));
				visionS = new Vision(new VisionIO() {
				}, new VisionIO() {
				},
						new VisionIO() {
						}, new VisionIO() {
						}); // MUST be same number of cameras as in real robot
		}
		drivetrainS.resetPose(FieldConstants.START_POSE);
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		Pathfinding.setPathfinder(new LocalADStarAK());
		NamedCommands.registerCommands(autoCommands);
		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}
		PathfindingCommand.warmupCommand()
				.andThen(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0))
				.finallyDo(() -> RobotContainer.field.getObject("target pose")
						.setPose(new Pose2d(-50, -50, new Rotation2d())))
				.schedule();
		if (!AutoBuilder.isConfigured()) {
			throw new RuntimeException(
					"AutoBuilder was not configured before attempting to build an auto chooser");
		}
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		if (drivetrainS instanceof Swerve) {
			Command orientBeforeData = ((Swerve) drivetrainS).orientModules(Swerve.getCircleOrientations());
			autoChooser.addOption("Wheel Radius Characterization",
					orientBeforeData
							.andThen(new WheelRadiusCharacterization(drivetrainS,
									WheelRadiusCharacterization.Direction.CLOCKWISE))
							.withName("DRIVE wheel radius characterization"));
		} else {
			autoChooser.addOption("Wheel Radius Characterization",
					new WheelRadiusCharacterization(drivetrainS,
							WheelRadiusCharacterization.Direction.CLOCKWISE)
							.withName("DRIVE wheel radius characterization"));
		}
		autoChooser.addOption("Drive Static Characterization",
				new StaticCharacterization(drivetrainS, drivetrainS::runCharacterization,
						drivetrainS::getCharacterizationVelocity)
						.finallyDo(drivetrainS::endCharacterization)
						.withName("Drive Static Characterization"));
		autoChooser.addOption("Drive FeedForward Characterization",
				new FeedForwardCharacterization(drivetrainS, drivetrainS::runCharacterization,
						drivetrainS::getCharacterizationVelocity, () -> false) // NEVER automatically end. MUST disable to end.
						.finallyDo(drivetrainS::endCharacterization)
						.withName("Drive FeedForward Characterization"));
		autoChooser.addOption("FUNI SONG", new OrchestraC("mii"));
		SmartDashboard.putData(field);
		// Store the last known value of autoChooser.get()
		final Command[] lastAuto = { autoChooser.get() };

		new Thread(() -> {
			while (true) {
				try {
					// Get the current value from autoChooser
					Command currentAutoValue = autoChooser.get();

					// Check if the value has changed
					if (currentAutoValue != null) {
						if (!currentAutoValue.equals(lastAuto[0])) {
							// Update the last known value
							lastAuto[0] = currentAutoValue;
							// Run your logic
							try {
								currentAuto = currentAutoValue;
								Logger.recordOutput("RobotState/autoPath",
										PathFinder.parseAutoToPose2dList(currentAutoValue.getName())
												.toArray(Pose2d[]::new));
								field.getObject("path")
										.setPoses(PathFinder.parseAutoToPose2dList(currentAutoValue.getName()));
							} catch (Exception e) {
								System.err.println("NO FOUND PATH FOR DESIRED AUTO!!");
								field.getObject("path").setPoses(
										new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
												new Pose2d(-50.2, -50, new Rotation2d())
										});
							}
						}
					}

					// Sleep for a short duration to prevent excessive CPU usage
					Thread.sleep(100); // Adjust the interval as necessary
				} catch (InterruptedException e) {
					Thread.currentThread().interrupt();
					System.err.println("Polling thread interrupted");
					break;
				}
			}
		}).start();
		// Configure the trigger bindings
		configureBindings();
		addNTCommands();
	}

	public Optional<Rotation2d> getRotationTargetOverride() {
		// Some condition that should decide if we want to override rotation
		return angleOverrider;
	}

	private void configureBindings() {
		xButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.onTrue(new InstantCommand(() -> {
					System.out.println("Zeroing Gyro");
					drivetrainS.zeroHeading();
					drivetrainS.resetPose(FieldConstants.START_POSE);
				}));
		// Example Drive To 2024 Amp Pose, Bind to what you need.
		yButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(
						new Pose2d(3, 5.6,
								new Rotation2d(Units.degreesToRadians(0))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0));
		VisionConstants.Controls.autoIntake
				.whileTrue(new DriveToTargetUsingDriveAndAimAtPose(drivetrainS, Vision::updateNotePose,
				RobotContainer.visionS::objectVisionOkay, () -> false));
		if (Constants.currentMode == Mode.SIM) {
			// ButtonDrive.whileTrue(testOpponentRobot.getAutoCyleCommand());
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.get();
	}

	/**
	 * For SIMULATION ONLY, return the estimated current draw of the robot.
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {
		return new double[] { Math.min(drivetrainS.getCurrent(), 200)
		};
	}

	private static void addNTCommands() {
		SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
	}

	/**
	 * RUN EACH system's test command. Does NOT run any checks on vision.
	 * 
	 * @return a command with all of them in a sequence.
	 */
	public static Command allSystemsCheck() {
		return Commands.sequence(visionS.getSystemCheckCommand(),
				drivetrainS.getRunnableSystemCheckCommand());
	}

	public static HashMap<String, Double> combineMaps(
			List<HashMap<String, Double>> maps) {
		HashMap<String, Double> combinedMap = new HashMap<>();
		// Iterate over the list of maps
		for (HashMap<String, Double> map : maps) {
			combinedMap.putAll(map);
		}
		return combinedMap;
	}

	public static HashMap<String, Double> getAllTemps() {
		// List of HashMaps
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(), visionS.getTemps());
		// Combine all maps
		HashMap<String, Double> combinedMap = combineMaps(maps);
		return combinedMap;
	}

	/**
	 * Checks EACH system's status (DOES NOT RUN THE TESTS)
	 * 
	 * @return true if ALL systems were good.
	 */
	public static boolean allSystemsOK() {
		return drivetrainS
				.getTrueSystemStatus() == SubsystemChecker.SystemStatus.OK
				&& visionS.getSystemStatus() == SubsystemChecker.SystemStatus.OK;
	}

	public static Collection<ParentDevice> getOrchestraDevices() {
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		return devices;
	}

	public static Subsystem[] getAllSubsystems() {
		Subsystem[] subsystems = new Subsystem[2];
		subsystems[0] = drivetrainS;
		subsystems[1] = visionS;
		return subsystems;
	}

	public static void updateSimulationWorld() {
		if (fieldSimulation != null)
			fieldSimulation.updateSimulationWorld();
	}
}