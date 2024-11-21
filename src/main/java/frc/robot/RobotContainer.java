// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.StaticCharacterization;
import frc.robot.commands.auto.BranchAuto;
import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.state_space.DoubleJointedArmC;
import frc.robot.commands.state_space.ElevatorC;
import frc.robot.commands.state_space.SingleJointedArmC;
import frc.robot.commands.state_space.FlywheelC;
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
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIO;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSparkBase;
import frc.robot.subsystems.drive.Tank.TankIOTalonFX;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIO;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIOSim;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmIOTalon;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmS;
import frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder.DoubleJointedArmArmEncoderIO;
import frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder.DoubleJointedArmArmEncoderIOCANCoder;
import frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder.DoubleJointedArmArmEncoderIODutyCycle;
import frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder.DoubleJointedArmArmEncoderIOThriftyAbsolute;
import frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder.DoubleJointedArmElbowEncoderIO;
import frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder.DoubleJointedArmElbowEncoderIOCANCoder;
import frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder.DoubleJointedArmElbowEncoderIODutyCycle;
import frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder.DoubleJointedArmElbowEncoderIOThriftyAbsolute;
import frc.robot.subsystems.state_space.Elevator.ElevatorIO;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOSim;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.state_space.Elevator.ElevatorIOTalon;
import frc.robot.subsystems.state_space.Elevator.ElevatorS;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIO;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIOCANCoder;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIODutyCycle;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIOREVAbsolute;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIOThriftyAbsolute;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIO;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSpark;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOTalon;
import frc.robot.subsystems.state_space.Flywheel.FlywheelS;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIO;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIOCANCoder;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIODutyCycle;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIOREVAbsolute;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIOThriftyAbsolute;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIO;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOSim;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOSpark;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOTalon;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmS;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIO;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIOCANCoder;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIODutyCycle;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIOREVAbsolute;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIOThriftyAbsolute;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.AIRobotInSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.Crescendo2024FieldSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.MecanumDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.TankDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.GyroSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIONavX;
import frc.robot.utils.drive.Sensors.GyroIOPigeon2;
import frc.robot.utils.drive.Sensors.GyroIOSim;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.revrobotics.spark.SparkBase;

import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.state_space.StateSpaceConstants;
/**
 * This code depends on WPILib 2025, Choreo 2025, PhotonLib 2025, Studica,
 * Phoenix-6 2025 (non-replay), REVLib 2025, URCL, GrappleLib 2025, AKit 2025,
 * and PathplannerLib 2025.
 * IT WILL NOT WORK WITHOUT ANY OF THESE!
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	public static FlywheelS flywheelS;
	public static SingleJointedArmS armS;
	public static ElevatorS elevatorS;
	public static DoubleJointedArmS doubleJointedArmS;
	private final SendableChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
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

	public static GamePieceState currentGamePieceStatus = GamePieceState.NO_GAME_PIECE;
	public static boolean userDrive = true;
	// Simulation
	public static Crescendo2024FieldSimulation fieldSimulation = null;
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
				// remove the .traj from the name
				choreo = choreo.replace(".traj", "");
				try {
					List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(choreo);
					for (PathPlannerPath path : auto) {
						commands.add(new Pair<String, Command>("Branch" + path.name,
								new BranchAuto(path.name,
										new Pose2d(
												path.getPoint(path.getAllPathPoints().size() - 1).position,
												path.getGoalEndState().rotation()),
										path.getGoalEndState().velocity().magnitude())));
						System.out.println("Added Branch" + path.name);
					}
					auto.clear();
					auto = null;
				} catch (FileVersionException | IOException | ParseException | NullPointerException e) {
					e.printStackTrace();
				}

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
				autoCommands.addAll(Arrays.asList(
						// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
						// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
						new Pair<String, Command>("BranchGrabbingGamePiece",
								new BranchAuto("Shoot",
										new Pose2d(7.4, 5.8, new Rotation2d()), 4))
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));
				autoCommands.addAll(createBranches());
				//Control logic assumes CTRE for CTRE, REV or CTRE for REV. This is due to REVLib's 2025 changes to AbsoluteEncoders requiring a SPARK to run them.
				FlywheelIO flywheelIO = null;
				switch (StateSpaceConstants.Flywheel.motorVendor) {
					case CTRE_ON_RIO:
					case CTRE_ON_CANIVORE:
						flywheelIO = new FlywheelIOTalon();
						break;
					// We're on REV
					case NEO_SPARK_MAX:
					case VORTEX_SPARK_FLEX:
						flywheelIO = new FlywheelIOSpark();
						break;
					default:	
						throw new IllegalArgumentException("Unknown implementation type, please check StateSpaceConstants.java!");
				}
				FlywheelEncoderIO flywheelEncoder = null;
				switch (StateSpaceConstants.Flywheel.encoderType) {
					case CTRE:
						flywheelEncoder = new FlywheelEncoderIOCANCoder(StateSpaceConstants.Flywheel.kEncoderID,
								StateSpaceConstants.Flywheel.CANBus, "FlywheelEncoder",
								StateSpaceConstants.Flywheel.encoderGearing,
								StateSpaceConstants.Flywheel.encoderOffsetRotations,
								StateSpaceConstants.Flywheel.isEncoderInverted);
						break;
					case REV_ABSOLUTE:
						if (flywheelIO instanceof FlywheelIOSpark) {
							flywheelEncoder = new FlywheelEncoderIOREVAbsolute(
									(SparkBase) flywheelIO.getSelfCheckingHardware().get(0).getHardware(),
									StateSpaceConstants.Flywheel.encoderGearing,
									StateSpaceConstants.Flywheel.encoderOffsetRotations,
									StateSpaceConstants.Flywheel.isEncoderInverted);
						} else {
							throw new IllegalArgumentException("REV Absolute Encoders require a SPARK to run them!");
						}
						break;
					case THRIFTY_ABSOLUTE:
						flywheelEncoder = new FlywheelEncoderIOThriftyAbsolute(
								StateSpaceConstants.Flywheel.kEncoderID,
								StateSpaceConstants.Flywheel.encoderGearing,
								StateSpaceConstants.Flywheel.encoderOffsetRotations,
								StateSpaceConstants.Flywheel.isEncoderInverted);
						break;
					case DUTY_CYCLE:
						flywheelEncoder = new FlywheelEncoderIODutyCycle(
								StateSpaceConstants.Flywheel.kEncoderID,
								StateSpaceConstants.Flywheel.encoderGearing,
								StateSpaceConstants.Flywheel.encoderOffsetRotations,
								StateSpaceConstants.Flywheel.isEncoderInverted);
						break;
					case NO_ATTACHED_ENCODER:
						flywheelEncoder = null;
						// No absolute Encoder
						break;
					default:
						throw new IllegalArgumentException(
								"Unknown implementation type, please check StateSpaceConstants.java!");
				}
				//create flywheelS
				flywheelS = new FlywheelS(flywheelIO, flywheelEncoder);
				//throw away old copy of flywheelIO and encoder
				flywheelIO = null;
				flywheelEncoder = null;

				SingleJointedArmIO armIO = null;
				switch (StateSpaceConstants.SingleJointedArm.motorVendor) {
					case CTRE_ON_RIO:
					case CTRE_ON_CANIVORE:
						armIO = new SingleJointedArmIOTalon();
						break;
					case NEO_SPARK_MAX:
					case VORTEX_SPARK_FLEX:
						armIO = new SingleJointedArmIOSpark();
						break;
					default:
						throw new IllegalArgumentException("Unknown implementation type, please check StateSpaceConstants.java!");
						
				}
				SingleJointedArmEncoderIO singleJointedArmEncoder = null;
				switch (StateSpaceConstants.SingleJointedArm.encoderType) {
					case CTRE:
						singleJointedArmEncoder = new SingleJointedArmEncoderIOCANCoder(StateSpaceConstants.SingleJointedArm.kEncoderID,
								StateSpaceConstants.SingleJointedArm.CANBus,
								"SingleJointedArmEncoder", StateSpaceConstants.SingleJointedArm.encoderGearing,
								StateSpaceConstants.SingleJointedArm.encoderOffsetRotations,
								StateSpaceConstants.SingleJointedArm.isEncoderInverted);
						break;
					case REV_ABSOLUTE:
						if (armIO instanceof SingleJointedArmIOSpark) {
							singleJointedArmEncoder = new SingleJointedArmEncoderIOREVAbsolute(
									(SparkBase) armIO.getSelfCheckingHardware().get(0).getHardware(),
									StateSpaceConstants.SingleJointedArm.encoderGearing,
									StateSpaceConstants.SingleJointedArm.encoderOffsetRotations,
									StateSpaceConstants.SingleJointedArm.isEncoderInverted);
						} else {
							throw new IllegalArgumentException("REV Absolute Encoders require a SPARK to run them!");
						}
						break;
					case THRIFTY_ABSOLUTE:
						singleJointedArmEncoder = new SingleJointedArmEncoderIOThriftyAbsolute(
								StateSpaceConstants.SingleJointedArm.kEncoderID,
								StateSpaceConstants.SingleJointedArm.encoderGearing,
								StateSpaceConstants.SingleJointedArm.encoderOffsetRotations,
								StateSpaceConstants.SingleJointedArm.isEncoderInverted);
						break;
					case DUTY_CYCLE:
						singleJointedArmEncoder = new SingleJointedArmEncoderIODutyCycle(
								StateSpaceConstants.SingleJointedArm.kEncoderID,
								StateSpaceConstants.SingleJointedArm.encoderGearing,
								StateSpaceConstants.SingleJointedArm.encoderOffsetRotations,
								StateSpaceConstants.SingleJointedArm.isEncoderInverted);
						break;
					case NO_ATTACHED_ENCODER:
						singleJointedArmEncoder = null;
						// No absolute Encoder
						break;
					default:
						throw new IllegalArgumentException(
								"Unknown implementation type, please check StateSpaceConstants.java!");
				}
				armS = new SingleJointedArmS(armIO, singleJointedArmEncoder);
				//throw away old copy of armIO and encoder
				armIO = null;
				singleJointedArmEncoder = null;

				ElevatorIO elevatorIO = null;
				switch (StateSpaceConstants.Elevator.motorVendor) {
					case CTRE_ON_RIO:
					case CTRE_ON_CANIVORE:
					
						elevatorIO = new ElevatorIOTalon();
						break;
					case NEO_SPARK_MAX:
					case VORTEX_SPARK_FLEX:
						elevatorIO = new ElevatorIOSpark();
						break;
					default:
						throw new IllegalArgumentException("Unknown implementation type, please check StateSpaceConstants.java!");
				}
				ElevatorEncoderIO elevatorEncoder = null;
				switch (StateSpaceConstants.Elevator.encoderType) {
					case CTRE:
						elevatorEncoder = 
								new ElevatorEncoderIOCANCoder(StateSpaceConstants.Elevator.kEncoderID,
										StateSpaceConstants.SingleJointedArm.CANBus, "elevatorEncoder",StateSpaceConstants.Elevator.encoderGearing,
										StateSpaceConstants.Elevator.encoderOffsetRotations,
										StateSpaceConstants.SingleJointedArm.isEncoderInverted);
						break;
					case REV_ABSOLUTE:
						if (elevatorIO instanceof ElevatorIOSpark) {
							elevatorEncoder = new ElevatorEncoderIOREVAbsolute(
									(SparkBase) elevatorIO.getSelfCheckingHardware().get(0).getHardware(),
									StateSpaceConstants.Elevator.encoderGearing,
									StateSpaceConstants.Elevator.encoderOffsetRotations,
									StateSpaceConstants.Elevator.isEncoderInverted);
						} else {
							throw new IllegalArgumentException("REV Absolute Encoders require a SPARK to run them!");
						}
						break;
					case THRIFTY_ABSOLUTE:
						elevatorEncoder = new ElevatorEncoderIOThriftyAbsolute(
								StateSpaceConstants.Elevator.kEncoderID,
								StateSpaceConstants.Elevator.encoderGearing,
								StateSpaceConstants.Elevator.encoderOffsetRotations,
								StateSpaceConstants.Elevator.isEncoderInverted);
						break;
					case DUTY_CYCLE:
						elevatorEncoder = new ElevatorEncoderIODutyCycle(
								StateSpaceConstants.Elevator.kEncoderID,
								StateSpaceConstants.Elevator.encoderGearing,
								StateSpaceConstants.Elevator.encoderOffsetRotations,
								StateSpaceConstants.Elevator.isEncoderInverted);
						break;
					case NO_ATTACHED_ENCODER:
						elevatorEncoder = null;
						// No absolute Encoder
						break;
					default:
						throw new IllegalArgumentException(
								"Unknown implementation type, please check StateSpaceConstants.java!");
				}
				elevatorS = new ElevatorS(elevatorIO, elevatorEncoder);
				//throw away old copy of elevatorIO and encoder
				elevatorIO = null;
				elevatorEncoder = null;
				
				// Double Jointed Arm will always be CTRE for latency reasons
				switch (StateSpaceConstants.DoubleJointedArm.doubleJointedEncoderType) {
					case CTRE:
						doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOTalon(),
								new DoubleJointedArmArmEncoderIOCANCoder(StateSpaceConstants.DoubleJointedArm.kArmEncoderID,
										StateSpaceConstants.DoubleJointedArm.CANBus, "doubleJointedArmArmEncoder",
										StateSpaceConstants.DoubleJointedArm.armEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.armEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isArmEncoderInverted),
								new DoubleJointedArmElbowEncoderIOCANCoder(StateSpaceConstants.DoubleJointedArm.kElbowEncoderID,
										StateSpaceConstants.DoubleJointedArm.CANBus, "doubleJointedArmElbowEncoder",
										StateSpaceConstants.DoubleJointedArm.elbowEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.elbowEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isElbowEncoderInverted));
						break;
					case NO_ATTACHED_ENCODER:
						doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOTalon(), null, null);
						break;
					case DUTY_CYCLE:
						doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOTalon(),
								new DoubleJointedArmArmEncoderIODutyCycle(StateSpaceConstants.DoubleJointedArm.kArmEncoderID,
										StateSpaceConstants.DoubleJointedArm.armEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.armEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isArmEncoderInverted),
								new DoubleJointedArmElbowEncoderIODutyCycle(StateSpaceConstants.DoubleJointedArm.kElbowEncoderID,
										StateSpaceConstants.DoubleJointedArm.elbowEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.elbowEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isElbowEncoderInverted));
						break;
					case THRIFTY_ABSOLUTE:
						doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOTalon(),
								new DoubleJointedArmArmEncoderIOThriftyAbsolute(StateSpaceConstants.DoubleJointedArm.kArmEncoderID,
										StateSpaceConstants.DoubleJointedArm.armEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.armEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isArmEncoderInverted),
								new DoubleJointedArmElbowEncoderIOThriftyAbsolute(StateSpaceConstants.DoubleJointedArm.kElbowEncoderID,
										StateSpaceConstants.DoubleJointedArm.elbowEncoderGearing,
										Units.rotationsToRadians(StateSpaceConstants.DoubleJointedArm.elbowEncoderOffsetRotations),
										StateSpaceConstants.DoubleJointedArm.isElbowEncoderInverted));
						break;
					default:
						throw new IllegalArgumentException(
								"Due to REVLib 2025's changes to absolute encoders, sparkAnalog and REVAbsoluteEncoder are not supported for double jointed arms");

				}
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
						for (int i = 0; i < 4; i++){
							switch (DriveConstants.swerveModuleType){
								case SDSMK4I:
								moduleSimulations[i] = SwerveModuleSimulation
								.getMark4i(DriveConstants.getDriveTrainMotors(1),
										DriveConstants.getDriveTrainMotors(1),
										DriveConstants.kMaxDriveCurrent,
										DRIVE_WHEEL_TYPE.RUBBER, 2)
								.get();
								break;
								case THRIFTYSWERVE:
								moduleSimulations[i] = SwerveModuleSimulation.getThrifty(DriveConstants.getDriveTrainMotors(1),
								DriveConstants.getDriveTrainMotors(1),
								DriveConstants.kMaxDriveCurrent,
								DRIVE_WHEEL_TYPE.RUBBER, 2).get();
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
			flywheelS = new FlywheelS(new FlywheelIOSim(), new FlywheelEncoderIO(){});
			armS = new SingleJointedArmS(new SingleJointedArmIOSim(), new SingleJointedArmEncoderIO(){});
			elevatorS = new ElevatorS(new ElevatorIOSim(), new ElevatorEncoderIO(){});
			doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIOSim(),new DoubleJointedArmArmEncoderIO(){}, new DoubleJointedArmElbowEncoderIO(){});
				autoCommands.addAll(Arrays.asList(
						// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
						// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
						new Pair<String, Command>("SmartShoot", Commands.none()),
						new Pair<String, Command>("SmartIntake", Commands.none()),
						new Pair<String, Command>("BranchGrabbingGamePiece",
								new BranchAuto("Shoot",
										new Pose2d(7.4, 5.8, new Rotation2d()), 4))
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
				));

				autoCommands.addAll(createBranches());
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
				flywheelS = new FlywheelS(new FlywheelIO(){}, new FlywheelEncoderIO(){});
							armS = new SingleJointedArmS(new SingleJointedArmIO(){}, new SingleJointedArmEncoderIO(){});
							elevatorS = new ElevatorS(new ElevatorIO(){}, new ElevatorEncoderIO(){});
							doubleJointedArmS = new DoubleJointedArmS(new DoubleJointedArmIO(){}, new DoubleJointedArmArmEncoderIO(){}, new DoubleJointedArmElbowEncoderIO(){});
				autoCommands.addAll(Arrays.asList(
						// new Pair<String, Command>("AimAtAmp",new AimToPose(drivetrainS, new
						// Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0))))),
						new Pair<String, Command>("BranchGrabbingGamePiece",
								new BranchAuto("Shoot",
										new Pose2d(7.4, 5.8, new Rotation2d()), 4))
				// new Pair<String, Command>("BotAborter", new BotAborter(drivetrainS)), //NEEDS
				// A WAY TO KNOW WHEN TO ABORT FOR THE EXAMPLE AUTO!!!
				// new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new
				// Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
				// new Pair<String, Command>("PlayMiiSong", new OrchestraC("mii")),
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
		flywheelS.setDefaultCommand(new FlywheelC(flywheelS));
		armS.setDefaultCommand(new SingleJointedArmC(armS));
		elevatorS.setDefaultCommand(new ElevatorC(elevatorS));
		doubleJointedArmS.setDefaultCommand(new DoubleJointedArmC(doubleJointedArmS));
		autoChooser = AutoBuilder.buildAutoChooser();
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
						drivetrainS::getCharacterizationVelocity).finallyDo(drivetrainS::endCharacterization)
						.withName("Drive FeedForward Characterization"));
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(auto -> {
			try {
				currentAuto = auto;
				Logger.recordOutput("RobotState/autoPath",
						PathFinder.parseAutoToPose2dList(auto.getName()).toArray(Pose2d[]::new));
				field.getObject("path")
						.setPoses(PathFinder.parseAutoToPose2dList(auto.getName()));
			} catch (Exception e) {
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
		// Example Drive To 2024 Amp Pose, Bind to what you need.
		yButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(
						new Pose2d(1.9, 7.7,
								new Rotation2d(Units.degreesToRadians(90))),
						() -> DriveConstants.pathConstraints, drivetrainS, false, 0));
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
		return autoChooser.getSelected();
	}

	/**
	 * For SIMULATION ONLY, return the estimated current draw of the robot.
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {
		return new double[] { Math.min(drivetrainS.getCurrent(), 200),
      flywheelS.getCurrent(),
			armS.getCurrent(),
			elevatorS.getCurrent(),
			doubleJointedArmS.getCurrent()
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
	return Commands.sequence(drivetrainS.getRunnableSystemCheckCommand(),flywheelS.getSystemCheckCommand(),armS.getSystemCheckCommand(),elevatorS.getSystemCheckCommand(),doubleJointedArmS.getSystemCheckCommand());
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
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(),
		flywheelS.getTemps(),
		armS.getTemps(),
		elevatorS.getTemps(),
		doubleJointedArmS.getTemps());

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
		return drivetrainS.getTrueSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& flywheelS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& elevatorS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& armS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& doubleJointedArmS.getSystemStatus() == SubsystemChecker.SystemStatus.OK;
	 }
	public static Collection<ParentDevice> getOrchestraDevices() {
    
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		devices.addAll(flywheelS.getOrchestraDevices());
		devices.addAll(elevatorS.getOrchestraDevices());
    	devices.addAll(armS.getOrchestraDevices());
		devices.addAll(doubleJointedArmS.getOrchestraDevices());

    return devices;
	}
	public static Subsystem[] getAllSubsystems(){
		Subsystem[] subsystems = new Subsystem[5];
		subsystems[0] = drivetrainS;
    	subsystems[1] = flywheelS;
    	subsystems[2] = elevatorS;
    	subsystems[3] = armS;
    	subsystems[4] = doubleJointedArmS;
		return subsystems;
	}

	public static void updateSimulationWorld() {
		if (fieldSimulation != null)
			fieldSimulation.updateSimulationWorld();
	}
}
