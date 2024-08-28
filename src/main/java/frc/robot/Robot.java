// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.SysIdRoutines;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	public static boolean isRed;
	private boolean isPracticeDSMode = false;
	private double lastMatchTime = 0;
	public static SysIdRoutines runningTest = Constants.SysIdRoutines
			.values()[0];
	private static final List<PeriodicFunction> periodicFunctions = new ArrayList<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		if (!Constants.isCompetition) {
			PortForwarder.add(22, "orangepi@photonvision.local", 22);
			PortForwarder.add(22, "photonvision.local", 22);
		}
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard
		//TODO: CONFIRM AKIT logs and SignalLogger logs are going to the USB stick
		Logger.recordMetadata("ProjectName", "The Chef"); // Set a metadata value
		Logger.recordMetadata("TuningMode",
				Boolean.toString(Constants.isTuningPID));
		Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
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
		Logger.registerURCL(URCL.startExternal(Constants.manCanIdsToNames()));
		Logger.start();
		m_robotContainer = new RobotContainer();
		DataHandler.startHandler("C:");
		SmartDashboard.putString("QUEUED TEST", runningTest.toString()); //Put what Test we're going to run on the test controller.
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
		double startTime = Logger.getRealTimestamp();
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			DriveConstants.pathConstraints = new PathConstraints(
				DriveConstants.pathConstraints.getMaxVelocityMps(),
					DriveConstants.maxTranslationalAcceleration.get(),
					DriveConstants.pathConstraints.getMaxAngularVelocityRps(),
					DriveConstants.maxRotationalAcceleration.get());
			DriveConstants.moduleLimitsFree = new ModuleLimits(
					DriveConstants.kMaxSpeedMetersPerSecond,
					DriveConstants.maxTranslationalAcceleration.get(),
					DriveConstants.maxRotationalAcceleration.get());
		}, DriveConstants.maxTranslationalAcceleration,
				DriveConstants.maxRotationalAcceleration);
		DataHandler.updateHandlerState();
		SmartDashboard.putString("Match State",
				Constants.currentMatchState.name());
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		runningTest = Constants.SysIdRoutines
				.values()[RobotContainer.currentTest];
		SmartDashboard.putString("QUEUED TEST", runningTest.toString());
		CommandScheduler.getInstance().run();
		for (PeriodicFunction f : periodicFunctions) {
			f.runIfReady();
		}
		Logger.recordOutput("MemoryTotal", Runtime.getRuntime().totalMemory());
        Logger.recordOutput("MemoryFree", Runtime.getRuntime().freeMemory());
		SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
		Logger.recordOutput("BatteryVoltage",
				RobotController.getBatteryVoltage());
		CANBus.CANBusStatus canBusStatus = CANBus.getStatus("rio");
		Logger.recordOutput("CANUtil", canBusStatus.BusUtilization * 100.0);
		double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
		Logger.recordOutput("RobotPeriodicMS", runtimeMS);
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		isPracticeDSMode = false;
		if (Constants.currentMatchState == FRCMatchState.ENDGAME) {
			Constants.currentMatchState = FRCMatchState.MATCHOVER;
		} else {
			Constants.currentMatchState = FRCMatchState.DISABLED;
		}
	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		Constants.currentMatchState = FRCMatchState.AUTOINIT;
		RobotContainer.drivetrainS.zeroHeading();

		RobotContainer.drivetrainS.zeroHeading(); //ENSURE gyro is reset.
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			if (Constants.currentMode == frc.robot.Constants.Mode.SIM){
				if (RobotContainer.currentAuto != null) {
					RobotContainer.fieldSimulation.resetField(true);
					RobotContainer.fieldSimulation.getMainDriveSimulation().setSimulationWorldPose(PathPlannerAuto.getStaringPoseFromAutoFile(RobotContainer.currentAuto.getName()));
					RobotContainer.fieldSimulation.getMainDriveSimulation().resetOdometryToActualRobotPose();
				}
			}
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		Constants.currentMatchState = FRCMatchState.AUTO;
	}

	@Override
	public void teleopInit() {
		Constants.currentMatchState = FRCMatchState.TELEOPINIT;
		RobotContainer.field.getObject("path").setTrajectory(new Trajectory());
		RobotContainer.field.getObject("target pose")
				.setPose(new Pose2d(-50, -50, new Rotation2d())); //the void
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
		/*An FRC teleop period takes 2 minutes and 15 seconds (135 seconds). Endgame occurs during the last 20 seconds.
		Based on this, endgame should initialize at 115 seconds and end at 135 seconds. */
		double matchTime = DriverStation.getMatchTime();
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
			}
		} else {
			if (matchTime % 1 != 0) { //is a double (running on DS)
				if (matchTime < lastMatchTime) {
					isPracticeDSMode = true;
				}
				lastMatchTime = matchTime;
			}
			Constants.currentMatchState = FRCMatchState.TELEOP;
		}
		if (RobotContainer.driveController.getPOV() == 0) {
			//System.err.println("UP");
			DataHandler.logData(new double[] { 4.5, 25.4
			}, "shouldUpdateModel");
		}
		if (RobotContainer.manipController.getAButton()) {
			System.out.println("A");
			DataHandler.logData(new double[] { 4.5
			}, "modelInputs");
		}
	}

	@Override
	public void testInit() {
		Constants.currentMatchState = FRCMatchState.TESTINIT;
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		//RobotContainer.allSystemsCheck().schedule();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		for (Map.Entry<String, Double> set : RobotContainer.getAllTemps()
				.entrySet()) {
			Logger.recordOutput("Temps/" + set.getKey(), set.getValue());
		}
		Constants.currentMatchState = FRCMatchState.TEST;
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		/*We don't assign a match state for either simulation function because we most likely would want to test features that occur
		at different game periods like tele and auto in simulation*/
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
      RobotContainer.updateSimulationWorld();
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
				RobotContainer.getCurrentDraw()));
		SmartDashboard.putNumber("Robot Voltage",
				RobotController.getBatteryVoltage());
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
			if (Logger.getTimestamp() > lastRunTimeSeconds + periodSeconds) {
				callback.run();
				lastRunTimeSeconds = Logger.getTimestamp();
			}
		}
	}
}
