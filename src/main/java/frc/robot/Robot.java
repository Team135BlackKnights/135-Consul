// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.Elastic;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LogTimingReceiver;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.VirtualSubsystem;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedBattery;
import frc.robot.utils.Touchboard.PosePlotterUtil;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.DriveTrainType;
import frc.robot.utils.maths.TimeUtil;

/*
TODO: Setup TODOTree to go through the year-by-year updating checklist. 

Install TODOTree from here https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree
Once installed, open settings (gear icon at the very bottom),
click settings on the menu that appears, then click the sheet of paper icon
by the WPILIB logo. Copy and paste the "TODOTreeSetup.txt" file into the settings.json
file, and save. If the setup worked, the first line of this paragraph should be yellow.
Once that is done, click on the TODOTree extension icon (the one with the tree) and finish 
everything labeled "YEARLYUPDATE" in the tree.
*/
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;
	private static final double loopOverrunWarningTimeout = 0.05;
	private RobotContainer m_robotContainer;
	public static boolean isRed;
	private boolean isPracticeDSMode = false, loggerStarted = false, matchHasEnded = false;
	public static double matchTime = 0;
	private double lastMatchTime = 0, previousTime = Logger.getTimestamp(), accumulatedCharge = 0;
	@SuppressWarnings("unused")
	private static final List<PeriodicFunction> periodicFunctions = new ArrayList<>();
	public static final CANBus rioCanBus = CANBus.roboRIO();
	public static final CANBus everythingCanBus = new CANBus("everything");
	public static Pose3d elevatorPose = new Pose3d();
	public static Pose3d armPose = new Pose3d();
	public static Pose3d algaePose = new Pose3d();
	private boolean autoHasStarted = false, hasCalculatedAuto = false;
	private String oldAutoString = "";

	public Robot() {
		CanBridge.runTCP();
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		PortForwarder.add(5800, "10.1.35.11", 5800);
		PortForwarder.add(5800, "10.1.35.12", 5800);
		if (!Constants.isCompetition) {
		}
		// execute PushOrangePiCode.java
		/*
		 * System.out.println("Pushing code to Orange Pi");
		 * // run a new thread where we run the script to push code to the Orange Pi
		 * new Thread(() -> {
		 * try {
		 * // Check the current user
		 * ProcessBuilder whoamiPb = new ProcessBuilder("whoami");
		 * Process whoamiProcess = whoamiPb.start();
		 * BufferedReader reader = new BufferedReader(
		 * new InputStreamReader(whoamiProcess.getInputStream()));
		 * String user = reader.readLine();
		 * System.out.println("Current user: " + user);
		 * // Run the script with sudo
		 * ProcessBuilder pb = new ProcessBuilder("bash", "-c",
		 * "/home/lvuser/deploy/OrangePi/PushOrangePiCode.sh");
		 * pb.redirectErrorStream(true); // Redirect error stream to output stream
		 * Process process = pb.start();
		 * BufferedReader scriptOutputReader = new BufferedReader(
		 * new InputStreamReader(process.getInputStream()));
		 * String line;
		 * while ((line = scriptOutputReader.readLine()) != null) {
		 * System.out.println(line);
		 * }
		 * int exitCode = process.waitFor();
		 * System.out.println("Process exited with code: " + exitCode);
		 * } catch (Exception e) {
		 * e.printStackTrace();
		 * }
		 * }).start();
		 */
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard
		Logger.recordMetadata("ProjectName", "The Chef"); // Set a metadata value
		Logger.recordMetadata("TuningMode",
				Boolean.toString(TuningConstants.isTuningPID));
		Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		// CameraServer.startAutomaticCapture();

		// Sanity check for aprilTag fields (thx FIRST so cool),
		// I'm assuming worlds will be a welded field while Indiana is AndyMark
		//

		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}
		switch (Constants.currentMode) {
			case REAL:
				// Running on a real robot, log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				SignalLogger.setPath("/media/sda1/");
				break;
			case SIM:
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(
						new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}
		Logger.addDataReceiver(new LogTimingReceiver());
		Logger.start();
		SignalLogger.enableAutoLogging(false);
		try {
			Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
			watchdogField.setAccessible(true);
			Watchdog watchdog = (Watchdog) watchdogField.get(this);
			watchdog.setTimeout(loopOverrunWarningTimeout);
		} catch (Exception e) {
			DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
		}
		var mathShared = MathSharedStore.getMathShared();
		MathSharedStore.setMathShared(
				new MathShared() {
					@Override
					public void reportError(String error, StackTraceElement[] stackTrace) {
						if (error.startsWith("x and y components of Rotation2d are zero")) {
							// get rid of the stupid rotation2d warning
							return;
						}
						mathShared.reportError(error, stackTrace);
					}

					@Override
					public void reportUsage(MathUsageId id, int count) {
						mathShared.reportUsage(id, count);
					}

					@Override
					public double getTimestamp() {
						return mathShared.getTimestamp();
					}
				});
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			Logger.recordOutput(
					"Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
			Logger.recordOutput("Commands/CommandsAll/" + name, count > 0);
		};
		CommandScheduler.getInstance()
				.onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
		CommandScheduler.getInstance()
				.onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
		CommandScheduler.getInstance()
				.onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
		loggerStarted = true;
		m_robotContainer = new RobotContainer();
		// DataHandler.startHandler();
		for (Subsystem subsys : RobotContainer.getAllSubsystems()) {
			if (subsys instanceof SubsystemChecker) {
				((SubsystemChecker) subsys).allowFaultPolling(false);
			}
		}
		RobotController.setBrownoutVoltage(6.0);
		SmartDashboard.putBoolean("ShouldEndLog", false);
		// read the accumated charge from the last boot, so we can set it to that on
		// boot.
		accumulatedCharge = 0;
		// Publish the current mode of the robot (to check in pit display)
		Logger.recordOutput("SystemStatus/robotMode", Constants.currentMode);
		Elastic.selectTab("Disabled/Prematch");
		// Threads.setCurrentThreadPriority(true, 10); // Java magic to speed up loops.

	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Every 10 seconds, call the GC. Yes, CPU cycles, BUT it's better than running
		// out of memory.
		if (System.currentTimeMillis() % 10000 < 20) {
			System.gc();
		}
		Threads.setCurrentThreadPriority(true, 99); // Java magic to speed up loops.

		long currentTime = System.currentTimeMillis();

		if (Constants.logBatteryPercent) {
			// This is where we update the battery voltage and current draw. Converts
			// current draw (Amps) to Coloumbs.
			double deltaTime = currentTime - previousTime;
			previousTime = currentTime;
			// Calculate the charge used since the last update (this is borked by the new
			// AKit PDP)
			double chargeUsed = 0 * deltaTime;// pdh.getInstance().pdpTotalCurrent * deltaTime; //pdpTotalCurrent is the
												// total current draw in amps from the PDP AT THIS MOMENT!
			accumulatedCharge += chargeUsed;

			// Calculate the remaining charge percentage
			double batteryPercentage = 100 * (1 - (accumulatedCharge / (64800 - 4800))); // 64800 is the total charge of
																							// the
			// battery in Coloumbs (18 * 3600s/hr)
			batteryPercentage = Math.max(0, batteryPercentage); // Ensure it doesn't go below 0%
			Logger.recordOutput("SystemStatus/BatteryPercentage", batteryPercentage);
			Logger.recordOutput("SystemStatus/AccumulatedCharge", accumulatedCharge);
		}
		// Record the current accumated charge, so we can set it to that on next boot.
		Logger.recordOutput("FMS/isFMSAttached", DriverStation.isFMSAttached());
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			DriveConstants.pathConstraints = new PathConstraints(
					DriveConstants.pathConstraints.maxVelocityMPS(),
					DriveConstants.maxTranslationalAcceleration.get(),
					DriveConstants.pathConstraints.maxAngularVelocityRadPerSec(),
					DriveConstants.maxRotationalAcceleration.get());
			DriveConstants.moduleLimitsLow = new ModuleLimits(
					DriveConstants.kMaxSpeedMetersPerSecond,
					DriveConstants.maxTranslationalAcceleration.get(),
					DriveConstants.maxRotationalAcceleration.get());
		}, DriveConstants.maxTranslationalAcceleration,
				DriveConstants.maxRotationalAcceleration);
		// long dataStartTime = System.currentTimeMillis();
		// DataHandler.updateHandlerState();
		// Logger.recordOutput("SystemStatus/Periodic/OrangePiMS",
		// Math.abs(dataStartTime - System.currentTimeMillis()));
		Logger.recordOutput("MatchState", Constants.currentMatchState.name());
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		VirtualSubsystem.periodicAll();
		for (PeriodicFunction f : periodicFunctions) {
			f.runIfReady();
		}
		Logger.recordOutput("SystemStatus/MemoryTotal", Runtime.getRuntime().totalMemory());
		Logger.recordOutput("SystemStatus/MemoryFree", Runtime.getRuntime().freeMemory());
		matchTime = DriverStation.getMatchTime();
		Logger.recordOutput("MatchTime", matchTime);

		double batteryVoltage = (Constants.currentMode == Constants.Mode.SIM)
				? SimulatedBattery.getBatteryVoltage().baseUnitMagnitude()
				: RobotController.getBatteryVoltage();

		Logger.recordOutput("SystemStatus/BatteryVoltage", batteryVoltage);

		updateAdvantageScopePiecesLive();

		double runtimeMS = (System.currentTimeMillis() - currentTime);
		// long to double for the recordOutput
		Logger.recordOutput("SystemStatus/RobotPeriodicMS", runtimeMS);
		Threads.setCurrentThreadPriority(false, 10); // Return to normal thread priority (so when next loop comes, max

	}

	// YEARLYUPDATE: Grant explain what this does (Ur Mom -G)
	private void updateAdvantageScopePiecesLive() {

	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		// make sure we are on the correct side
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		isPracticeDSMode = false;
		if (Constants.currentMatchState == FRCMatchState.ENDGAME) {
			Constants.currentMatchState = FRCMatchState.MATCHOVER;
			// DataHandler.logData(VisionConstants.FieldConstants.aprilTagOffsets,"MatchAprilTagOffsets");
		} else {
			Constants.currentMatchState = FRCMatchState.DISABLED;
		}
		for (Subsystem subsys : RobotContainer.getAllSubsystems()) {
			if (subsys instanceof SubsystemChecker) {
				((SubsystemChecker) subsys).allowFaultPolling(false);
			}
		}
	}

	@Override
	public void disabledPeriodic() {
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		if (!autoHasStarted) {
			String[] auto = PosePlotterUtil.getAutoString().split("_");
			if (auto.length < 2) {
				auto = new String[] { "-120", "5.542", "NA" };
			}
			double x = 7.02;
			double y;
			if (Double.parseDouble(auto[1]) == Double.NaN) {
				y = 7.1;
			} else {
				y = DriveConstants.kBumperToBumperLength / 2 + (1 - Double.parseDouble(auto[1]))
						* (FieldConstants.FIELD_HEIGHT - DriveConstants.kBumperToBumperLength);
			}
			double theta = Units.degreesToRadians(Double.parseDouble(auto[0]));
			Pose2d startingPose = new Pose2d(x, y, new Rotation2d(theta));
			RobotContainer.drivetrainS.resetPose(GeomUtil.apply(startingPose, false));
			if (Constants.currentMode == frc.robot.Constants.Mode.SIM) {
				RobotContainer.fieldSimulation.getMainDriveSimulation()
						.setSimulationWorldPose(GeomUtil.apply(startingPose, false));
			}
			if (PosePlotterUtil.getAutoString() != null && !PosePlotterUtil.getAutoString().equals(oldAutoString)) {
				oldAutoString = PosePlotterUtil.getAutoString();
				PosePlotterUtil.calculateAuto();
				hasCalculatedAuto = true;
			}
		}
		if (loggerStarted && SmartDashboard.getBoolean("ShouldEndLog", false)) {
			Logger.recordOutput("EndedProperly", true);
			Logger.end();
			loggerStarted = false; // debonuces
			System.out.println("ENDING LOG");
		} else {
			{
				if (loggerStarted) {
					Logger.recordOutput("EndedProperly", false);
				}
			}
		}
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		// make sure we are on the correct side
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		Elastic.selectTab("Autonomous");
		autoHasStarted = true;
		Constants.currentMatchState = FRCMatchState.AUTOINIT;
		for (Subsystem subsys : RobotContainer.getAllSubsystems()) {
			if (subsys instanceof SubsystemChecker) {
				((SubsystemChecker) subsys).allowFaultPolling(false);
			}
		}

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			System.out.println(m_robotContainer.getAutoName());
			if (m_robotContainer.getAutoName().equals("DynamicPathing") && !hasCalculatedAuto) {
				System.out.println("Building Dynamic Auto");
				PosePlotterUtil.calculateAuto();
				hasCalculatedAuto = true;
			}
			if (Constants.currentMode == frc.robot.Constants.Mode.SIM) {
				RobotContainer.fieldSimulation.resetField(true);
				if (RobotContainer.currentAuto != null) {
					try {
						PathPlannerPath path = PathPlannerAuto
								.getPathGroupFromAutoFile(
										RobotContainer.currentAuto.getName())
								.get(0);
						if (DriveConstants.driveType == DriveTrainType.TANK) {
							RobotContainer.fieldSimulation.getMainDriveSimulation()
									.setSimulationWorldPose(path.getStartingDifferentialPose());
						} else {
							RobotContainer.fieldSimulation.getMainDriveSimulation()
									.setSimulationWorldPose(
											Robot.isRed ? FlippingUtil.flipFieldPose(new Pose2d(
													path.getPoint(0).position,
													path.getIdealStartingState().rotation()))
													: new Pose2d(
															path.getPoint(0).position,
															path.getIdealStartingState().rotation()));
						}
					} catch (Exception e) {

					}
					RobotContainer.fieldSimulation.getMainDriveSimulation()
							.resetOdometryToActualRobotPose();
					RobotContainer.fieldSimulation.resetField(true);
					RobotContainer.fieldSimulation.addPoints(3);
				}
			}
			matchHasEnded = false;
			System.out.println("Scheduling Auto: " + m_autonomousCommand.getName());
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		Constants.currentMatchState = FRCMatchState.AUTO;
	}

	@Override
	public void teleopInit() {
		// make sure we are on the correct side
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;

		autoHasStarted = true;
		Elastic.selectTab("Teleoperated");
		Constants.currentMatchState = FRCMatchState.TELEOPINIT;
		for (Subsystem subsys : RobotContainer.getAllSubsystems()) {
			if (subsys instanceof SubsystemChecker) {
				((SubsystemChecker) subsys).allowFaultPolling(false);
			}
		}
		RobotContainer.field.getObject("path").setTrajectory(new Trajectory());
		RobotContainer.field.getObject("target pose")
				.setPose(new Pose2d(-50, -50, new Rotation2d())); // the void
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putData(m_autonomousCommand);

		/*
		 * An FRC teleop period takes 2 minutes and 15 seconds (135 seconds). Endgame
		 * occurs during the last 20 seconds.
		 * Based on this, endgame should initialize at 115 seconds and end at 135
		 * seconds.
		 */
		matchTime = DriverStation.getMatchTime();
		if (RobotContainer.angleOverrider.isPresent()) {
			Logger.recordOutput("Odometry/AimGoal",
					new Pose2d(RobotContainer.drivetrainS.getPose().getTranslation(),
							RobotContainer.angleOverrider.get()));
		} else {
			Logger.recordOutput("Odometry/AimGoal",
					RobotContainer.drivetrainS.getPose());
		}
		if (DriverStation.isFMSAttached() || isPracticeDSMode) {
			if (matchTime > 30) {
				Constants.currentMatchState = FRCMatchState.TELEOP;
			} else if (matchTime == 30) {
				Constants.currentMatchState = FRCMatchState.ENDGAMEINIT;
			} else if (matchTime < 30 && matchTime > 0) {
				Constants.currentMatchState = FRCMatchState.ENDGAME;
				Elastic.selectTab("Endgame");
			}
		} else {
			if (matchTime % 1 != 0) { // is a double (running on DS)
				if (matchTime < lastMatchTime) {
					isPracticeDSMode = true;
				}
				lastMatchTime = matchTime;
			}
			Constants.currentMatchState = FRCMatchState.TELEOP;
		}
		/*
		 * if (RobotContainer.driveController.getPOV() == 0) {
		 * //System.err.println("UP");
		 * DataHandler.logData(new double[] { 4.5, 25.4
		 * }, "shouldUpdateModel");
		 * }
		 * if (RobotContainer.manipController.getAButton()) {
		 * System.out.println("A");
		 * DataHandler.logData(new double[] { 4.5
		 * }, "modelInputs");
		 * }
		 */
	}

	@Override
	public void testInit() {
		Constants.currentMatchState = FRCMatchState.TESTINIT;
		Elastic.selectTab("Testing");
		for (Subsystem subsys : RobotContainer.getAllSubsystems()) {
			if (subsys instanceof SubsystemChecker) {
				((SubsystemChecker) subsys).allowFaultPolling(true);
			}
		}
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		// CommandScheduler.getInstance().schedule(RobotContainer.allSystemsCheck());
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		Constants.currentMatchState = FRCMatchState.TEST;
		long statusCalls = System.currentTimeMillis();
		CANBusStatus rioCanBusStatus = rioCanBus.getStatus();
		CANBusStatus driveCanBusStatus = DriveConstants.driveCanBus.getStatus();
		Logger.recordOutput("SystemStatus/CANMs", Math.abs(statusCalls -
				System.currentTimeMillis()));
		Logger.recordOutput("SystemStatus/CANUtil", rioCanBusStatus.BusUtilization *
				100.0);
		Logger.recordOutput("SystemStatus/DriveCANUtil",
				driveCanBusStatus.BusUtilization * 100.0);
		for (Map.Entry<String, Double> set : RobotContainer.getAllTemps()
				.entrySet()) {
			Logger.recordOutput("Temps/" + set.getKey(), set.getValue());
		}
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		/*
		 * We don't assign a match state for either simulation function because we most
		 * likely would want to test features that occur
		 * at different game periods like tele and auto in simulation
		 */
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		if (Constants.currentMode != frc.robot.Constants.Mode.REPLAY) {
			RobotContainer.updateSimulationWorld();
			if (Constants.currentMatchState == Constants.FRCMatchState.MATCHOVER) {
				if (!matchHasEnded) {
					RobotContainer.fieldSimulation.addPoints(2);
					matchHasEnded = true;
				}
			}
			Logger.recordOutput("Scoring/SimMatchOver", matchHasEnded);
		}

	}

	public static void addPeriodic(Runnable callback, double period) {
		periodicFunctions.add(new PeriodicFunction(callback, period));
	}

	private static class PeriodicFunction {
		private final Runnable callback;
		private final double periodSeconds;
		private double lastRunTimeSeconds;

		private PeriodicFunction(Runnable callback, double periodSeconds) {
			this.callback = callback;
			this.periodSeconds = periodSeconds;
			this.lastRunTimeSeconds = 0.0;
		}

		private void runIfReady() {
			if (TimeUtil.getRealTimeSeconds() > lastRunTimeSeconds + periodSeconds) {
				callback.run();
				lastRunTimeSeconds = TimeUtil.getRealTimeSeconds();
			}
		}
	}
}
