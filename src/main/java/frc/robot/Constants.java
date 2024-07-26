// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;

/**
 * Anything for the e n t i r e robot goes here. Specific constants go under
 * Utils, and their respective folder for Merge purposes.
 */
public final class Constants {
	public static final Mode currentMode;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,
		/** Running a physics simulator. */
		SIM,
		/** Replaying from a log file. */
		REPLAY
	}

	static {
		currentMode = Mode.REAL;
		//if (Robot.isReal()) currentMode = Mode.REAL;
		//else if (Robot.isSimulation()) currentMode = Mode.SIM;
		//else currentMode = Mode.REPLAY;
	}
	//FRCMatchState of the robot
	public static boolean isCompetition = false;
	public static FRCMatchState currentMatchState = FRCMatchState.DISABLED;
	public static boolean isTuningPID = true;

	/**
	 * Allows the robot to utilize switch statements to efficiently figure out
	 * the match period it's in. The current FRCMatchState is stored in
	 * currentMatchState. Use .name() to get the FRCMatchState as a String
	 */
	public static enum FRCMatchState {
		//Very beginning of test mode
		TESTINIT,
		//When test mode is running
		TEST,
		//Very beginning of auto
		AUTOINIT,
		//Throughout auto
		AUTO,
		//Very beginning of Teleop (Until Endgame)
		TELEOPINIT,
		//Throughout Teleop (Until Endgame)
		TELEOP,
		//Very beginning of Endgame (20 seconds left)
		ENDGAMEINIT,
		//Throughout Endgame
		ENDGAME,
		//When robot is disabled (Default State)
		DISABLED,
		//Runs when the match is over (after endgame)
		MATCHOVER
	}

	public static enum SysIdRoutines { swerveDrive }

	public static int PowerDistributionID = 1;

	//put datalog constants IN THE UTIL FOR THAT FILE. 
	public static Map<Integer, String> manCanIdsToNames() {
		HashMap<Integer, String> map = new HashMap<>();
		//this has to be adjusted for every block branch added!
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

	public static class DriveSimConstants {
		//id 1 is topmost leftmost. goes in order down, right.
		//Speaker translations
		public static Translation3d blueShootLocation = new Translation3d(0.225,
				5.55, 2.1);
		public static Translation3d redShootLocation = new Translation3d(16.317,
				5.55, 2.1); //in meters!
		public static double shotSpeed = 15;
		public static double intakeSpeed = 3;
		//Launcher position compared to the robot
		public static Transform3d launcherTransform = new Transform3d(0.292, 0,
				0.1225, new Rotation3d(0, 0, 0.0));
		public static Pose3d[] fieldPieceTranslations = new Pose3d[] {
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(228),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center up 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(294),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center up 2
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(96),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center down 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(30),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //center down 2
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BLUE CENTER
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BLUE CENTER + 1
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //BUE TOP 
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED CENTER
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED CENTER + 1
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), //RED TOP 
		};
	}
}