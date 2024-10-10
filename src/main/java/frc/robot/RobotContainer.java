// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.commands.auto.BranchAuto;
import frc.robot.commands.drive.DrivetrainC;
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
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIOTalonFX;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.utils.RunTest;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.MecanumDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.OpponentRobotSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.TankDriveSimulation;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;
import frc.robot.utils.drive.Sensors.GyroIOSim;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.subsystems.leds.LEDs;
import frc.robot.utils.leds.LEDConstants.ImageStates;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	private static final LEDs leds = new LEDs();
	private final SendableChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			yButtonDrive = new JoystickButton(driveController, 4), //used for Aim/Drive to pose
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
	
	public enum GamePieceState {
		NO_GAME_PIECE, HAS_NOTE, ABORT
	}
	public static GamePieceState currentGamePieceStatus = GamePieceState.NO_GAME_PIECE;

	// Simulation
	public static Crescendo2024FieldSimulation fieldSimulation = null;
	private OpponentRobotSimulation testOpponentRobot = null;
	public static Command currentAuto;

	/**
	 * Reads every Choreo file in the deploy folder and creates a command for
	 * each Checks within Filesystem.getDeployDirectory(), "choreo/" for all
	 * files NOT having two . in the name (including the one . in .traj)
	 * 
	 * @return
	 */
	private Collection<Pair<String, Command>> createBranches() {
		Collection<Pair<String, Command>> commands = new ArrayList<>();
		File choreoDirectory = new File(Filesystem.getDeployDirectory(),
				"choreo/");
		for (String choreo : choreoDirectory.list()) {
			// count number of . in the name using regex
			int dotCount = choreo.split("\\.", -1).length - 1;
			if (choreo.contains(".traj") && dotCount == 1) {
				//remove the .traj from the name
				choreo = choreo.replace(".traj", "");
				PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(choreo);
				Pose2d endPose = path.getPathPoses().get(path.getPathPoses().size() - 1);
				if (DriveConstants.kEndingPoses.containsKey(choreo)){
					endPose = DriveConstants.kEndingPoses.get(choreo);
				}
				commands.add(new Pair<String, Command>("Branch" + choreo,
						new BranchAuto(choreo,
								endPose,
								1)));
				System.out.println("Added Branch" + choreo);
				//kill the path to save memory
				path = null;
			}
		}
		return commands;
	}

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands. y * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
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
						drivetrainS = new Swerve(new GyroIONavX(),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
						break;
					case PIGEON:
						drivetrainS = new Swerve(new GyroIOPigeon2(),
								new ModuleIOKrakenFOC(0), new ModuleIOKrakenFOC(1),
								new ModuleIOKrakenFOC(2), new ModuleIOKrakenFOC(3));
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
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
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
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			//Placeholder values
			default:
				throw new IllegalArgumentException(
						"Unknown implementation type, please check DriveConstants.java!");
			}
			autoCommands.addAll(Arrays.asList(
					//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
					new Pair<String, Command>("BranchGrabbingGamePiece",
							new BranchAuto("Shoot",
									new Pose2d(7.4, 5.8, new Rotation2d()), 4))
			//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
			//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
			));
			autoCommands.addAll(createBranches());
			break;
		case SIM:
			switch (DriveConstants.driveType) {
			case SWERVE:
				final ModuleIOSim frontLeft = new ModuleIOSim(0),
						frontRight = new ModuleIOSim(1),
						backLeft = new ModuleIOSim(2), backRight = new ModuleIOSim(3);
				final GyroIOSim gyroIOSim = new GyroIOSim();
				drivetrainS = new Swerve(gyroIOSim, frontLeft, frontRight, backLeft,
						backRight);
				fieldSimulation = new Crescendo2024FieldSimulation(
						new SwerveDriveSimulation(DriveConstants.mainRobotProfile,
								gyroIOSim, frontLeft, frontRight, backLeft, backRight,
								drivetrainS.getKinematics(), FieldConstants.START_POSE,
								drivetrainS::resetPose));
				fieldSimulation.placeGamePiecesOnField(true);
				testOpponentRobot = new OpponentRobotSimulation(0);
				fieldSimulation.addRobot(testOpponentRobot);
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				final GyroIOSim tankGyroIOSim = new GyroIOSim();
				TankIOSim tankIOSim = new TankIOSim(tankGyroIOSim);
				drivetrainS = new Tank(tankIOSim);
				fieldSimulation = new Crescendo2024FieldSimulation(
						new TankDriveSimulation(DriveConstants.mainRobotProfile,
								tankGyroIOSim,
								new DifferentialDriveKinematics(
										DriveConstants.kChassisWidth),
								FieldConstants.START_POSE, (Tank) drivetrainS,
								tankIOSim, drivetrainS::resetPose));
				fieldSimulation.placeGamePiecesOnField(true);
				testOpponentRobot = new OpponentRobotSimulation(0);
				fieldSimulation.addRobot(testOpponentRobot);
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			default:
				final GyroIOSim mecanumGyroIOSim = new GyroIOSim();
				MecanumIOSim mecanumIOSim = new MecanumIOSim(mecanumGyroIOSim);
				drivetrainS = new Mecanum(mecanumIOSim);
				fieldSimulation = new Crescendo2024FieldSimulation(
						new MecanumDriveSimulation(DriveConstants.mainRobotProfile,
								mecanumGyroIOSim,
								new MecanumDriveKinematics(
										DriveConstants.kModuleTranslations[0],
										DriveConstants.kModuleTranslations[1],
										DriveConstants.kModuleTranslations[2],
										DriveConstants.kModuleTranslations[3]),
								FieldConstants.START_POSE, (Mecanum) drivetrainS,
								mecanumIOSim, drivetrainS::resetPose));
				fieldSimulation.placeGamePiecesOnField(true);
				testOpponentRobot = new OpponentRobotSimulation(0);
				fieldSimulation.addRobot(testOpponentRobot);
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			}
			autoCommands.addAll(Arrays.asList(
					//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
					new Pair<String, Command>("BranchGrabbingGamePiece",
							new BranchAuto("Shoot",
									new Pose2d(7.4, 5.8, new Rotation2d()), 4))
			//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
			//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
			));
			autoCommands.addAll(createBranches());
			break;
		default:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIO() {},
						new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				drivetrainS = new Tank(new TankIO() {});
				break;
			case MECANUM:
				drivetrainS = new Mecanum(new MecanumIO() {});
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
			}
			autoCommands.addAll(Arrays.asList(
					//new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
					new Pair<String, Command>("BranchGrabbingGamePiece",
							new BranchAuto("Shoot",
									new Pose2d(7.4, 5.8, new Rotation2d()), 4))
			//new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
			//new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
			));
			autoCommands.addAll(createBranches());
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
		if (!leds.gifFound(ImageStates.debug)) {
			Logger.recordOutput("LEDS/Main",
					"No images found for " + ImageStates.debug.name());
		}
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(auto -> {
			try {
				currentAuto = auto;
				field.getObject("path")
						.setPoses(PathFinder.parseAutoToPose2dList(auto.getName()));
			}
			catch (Exception e) {
				System.err.println("NO FOUND PATH FOR DESIRED AUTO!!");
				field.getObject("path").setPoses(
						new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
								new Pose2d(-50.2, -50, new Rotation2d())
				});
			}
		});
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
				.onTrue(new InstantCommand(() -> drivetrainS.zeroHeading()));
		yButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, true, drivetrainS));
		bButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, true, drivetrainS));
		aButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, false, drivetrainS));
		xButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, false, drivetrainS));
		//Example Drive To 2024 Amp Pose, Bind to what you need.
		yButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0));
		//swerve DRIVE tests
		//When user hits right bumper, go to next test, or wrap back to starting test for SysID.
		rightBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == Constants.SysIdRoutines.values().length - 1) {
				currentTest = 0;
				System.out.println("looping");
			} else {
				currentTest++;
			}
		}));
		//When user hits left bumper, go to next test, or wrap back to starting test for SysID.
		leftBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == 0) {
				currentTest = Constants.SysIdRoutines.values().length - 1;
				System.out.println("looping");
			} else {
				currentTest--;
			}
		}));
		//When using CTRE, be sure to hit Start so that the motors are logged via CTRE (For SysId)
		selectButtonTest.onTrue(Commands.runOnce(SignalLogger::stop));
		startButtonTest.onTrue(Commands.runOnce(SignalLogger::start));
		if (Constants.currentMode == Mode.SIM) {
			//ButtonDrive.whileTrue(testOpponentRobot.getAutoCyleCommand());
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
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
		return Commands.sequence(drivetrainS.getRunnableSystemCheckCommand(),
				leds.getSystemCheckCommand());
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
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps());
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
				&& leds.getSystemStatus() == SubsystemChecker.SystemStatus.OK;
	}

	public static Collection<ParentDevice> getOrchestraDevices() {
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		return devices;
	}

	public static Subsystem[] getAllSubsystems() {
		Subsystem[] subsystems = new Subsystem[1];
		subsystems[0] = drivetrainS;
		return subsystems;
	}

	public static void updateSimulationWorld() {
		if (fieldSimulation != null)
			fieldSimulation.updateSimulationWorld();
	}
}