// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import frc.robot.utils.drive.DriveConstants;

/**
 * Anything for the e n t i r e robot goes here. Specific constants go under
 * Utils, and their respective folder for Merge purposes.
 */
public final class Constants {
	public static Mode currentMode;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,
		/** Running a physics simulator. */
		SIM,
		/** Replaying from a log file. */
		REPLAY
	}

	// FRCMatchState of the robot
	public static final boolean isCompetition = false;
	public static FRCMatchState currentMatchState = FRCMatchState.DISABLED;
	public static final boolean logBatteryPercent = false;
	static {
		// MUST BE "AT EVENT" TO REPLAY!
		if (isCompetition) {
			if (Robot.isReal()) {
				currentMode = Mode.REAL;
			} else {
				currentMode = Mode.REPLAY;
			}
		} else {
			if (Robot.isReal()) {
				currentMode = Mode.REAL;
			}
			if (Robot.isSimulation()) {
				currentMode = Mode.SIM;
			}
		}
	}

	/**
	 * Allows the robot to utilize switch statements to efficiently figure out
	 * the match period it's in. The current FRCMatchState is stored in
	 * currentMatchState. Use .name() to get the FRCMatchState as a String
	 */
	public static enum FRCMatchState {
		// Very beginning of test mode
		TESTINIT,
		// When test mode is running
		TEST,
		// Very beginning of auto
		AUTOINIT,
		// Throughout auto
		AUTO,
		// Very beginning of Teleop (Until Endgame)
		TELEOPINIT,
		// Throughout Teleop (Until Endgame)
		TELEOP,
		// Very beginning of Endgame (20 seconds left)
		ENDGAMEINIT,
		// Throughout Endgame
		ENDGAME,
		// When robot is disabled (Default State)
		DISABLED,
		// Runs when the match is over (after endgame)
		MATCHOVER
	}

	public static enum EncoderType {
		NO_ATTACHED_ENCODER,
		DUTY_CYCLE,
		CTRE,
		THRIFTY_ABSOLUTE,
		REV_ABSOLUTE,
		REV_ANALOG
	}

	public static int PowerDistributionID = 1;

	// put datalog constants IN THE UTIL FOR THAT FILE.
	public static Map<Integer, String> manCanIdsToNames() {
		HashMap<Integer, String> map = new HashMap<>();
		// this has to be adjusted for every block branch added!
		map.put(DriveConstants.kBackLeftDrivePort, "backLeftDrive");
		map.put(DriveConstants.kBackLeftTurningPort, "backLeftTurn");
		map.put(DriveConstants.kBackRightDrivePort, "backRightDrive");
		map.put(DriveConstants.kBackRightTurningPort, "backRightTurn");
		map.put(DriveConstants.kFrontLeftDrivePort, "frontLeftDrive");
		map.put(DriveConstants.kFrontLeftTurningPort, "frontLeftTurn");
		map.put(DriveConstants.kFrontRightDrivePort, "frontRightDrive");
		map.put(DriveConstants.kFrontRightTurningPort, "frontRightTurn");
		return map;
	}

	public static class GeometryConstants {
		public static final double shotSpeed = 15;
		public static final double intakeSpeed = 3;
		public static double intakeOffset = Units.inchesToMeters(17.5);
		public static double ObjectDistanceZeroSpeed = Units.inchesToMeters(12);
		public static final Transform3d hopperStartTransform = new Transform3d(-Units.inchesToMeters(20),
				-Units.inchesToMeters(2), Units.inchesToMeters(40), new Rotation3d(0, 0, -Math.PI / 2));
		public static final Transform3d hopperMiddleTransform = new Transform3d(-Units.inchesToMeters(10),
				-Units.inchesToMeters(5), Units.inchesToMeters(36.25), new Rotation3d(0, 0, -Math.PI / 6));
		public static final Transform3d hopperTransform = new Transform3d(0, -Units.inchesToMeters(9),
				Units.inchesToMeters(36.25), new Rotation3d(0, 0, 0.0));
		// Intake position compared to the robot
		public static final Transform3d coralScorerTransform = new Transform3d(0.14, -0.015,
				0.16, new Rotation3d(0, Units.degreesToRadians(22), 0));
		//Placeholder!
			public static final Transform3d CoralDistanceFromCenter = new Transform3d(0,0, 0, new Rotation3d(0,0,0));

		
	}

	/**
	 * @apiNote arm and algae scorer are different
	 */
	public static class TuningConstants {
		public static final boolean isTuningPID = true,
				isTuningMacros = true,
				isTuningVision = true,
				isTuningModules = true,
				isTuningDrivetrain = true,
				isTuningCharacterization = true;
	}
}