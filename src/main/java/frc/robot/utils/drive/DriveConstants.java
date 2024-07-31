package frc.robot.utils.drive;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.CompetitionFieldUtils.Simulation.HolonomicChassisSimulation.RobotProfile;

public class DriveConstants {
	public static MotorVendor robotMotorController = MotorVendor.CTRE_ON_RIO;
	public static DriveTrainType driveType = DriveTrainType.SWERVE;
	public static GyroType gyroType = GyroType.PIGEON;

	/**
	 * What motors and motorContollers are we using
	 */
	public enum MotorVendor {
		NEO_SPARK_MAX, VORTEX_SPARK_FLEX, CTRE_ON_RIO, CTRE_ON_CANIVORE
	}

	/**
	 * The drivetrain type
	 */
	public enum DriveTrainType {
		SWERVE, TANK, MECANUM
	}

	/**
	 * The Gyro type
	 * 
	 * @apiNote NavX Swerve is untested.
	 */
	public enum GyroType {
		NAVX, PIGEON
	}

	public static final LoggableTunedNumber maxTranslationalAcceleration = new LoggableTunedNumber(
			"Drive/MaxTranslationalAcceleration", Units.feetToMeters(50));
	public static final LoggableTunedNumber maxRotationalAcceleration = new LoggableTunedNumber(
			"Drive/MaxRotationalAcceleration", 2 * Math.PI * 50);
	public static boolean fieldOriented = true;
	//135-Blocks was tested on a chassis with all CANSparkMaxes, as well as all Kraken-x60s.
	public static final double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
			kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
			kBumperToBumperWidth = Units.inchesToMeters(35.5), // Distance between bumpers
			kBumperToBumperLength = Units.inchesToMeters(35.5), // Distance between bumpers
			kDriveBaseRadius = Math.sqrt(
					kChassisLength * kChassisLength + kChassisWidth * kChassisWidth)
					/ 2,
			// Distance from center of robot to the farthest module
			kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1), //15.1
			kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
			// To find these set them to zero, then turn the robot on and manually set the
			// wheels straight.
			// The encoder values being read are then your new Offset values
			//REV Offsets 
			/*kFrontLeftAbsEncoderOffsetRad = 0.562867,
			kFrontRightAbsEncoderOffsetRad = 0.548137,
			kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
			kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861,*/
			//Ctre Offsets
			kFrontLeftAbsEncoderOffsetRad = 0, kFrontRightAbsEncoderOffsetRad = 0,
			kBackLeftAbsEncoderOffsetRad = 0, kBackRightAbsEncoderOffsetRad = 0,
			SKID_THRESHOLD = .5, //Meters per second
			MAX_G = .5;
	public static PathConstraints pathConstraints = new PathConstraints(
			kMaxSpeedMetersPerSecond, maxTranslationalAcceleration.get(),
			kMaxTurningSpeedRadPerSec, maxRotationalAcceleration.get());
	// kP = 0.1, kI = 0, kD = 0, kDistanceMultipler = .2; //for autoLock
	// Declare the position of each module
	public static final Translation2d[] kModuleTranslations = {
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2)
	};
	public static int kFrontLeftDrivePort = 16, // REV 16 CTRE 16
			kFrontLeftTurningPort = 17, // REV 16 CTRE 17
			kFrontLeftAbsEncoderPort = 20, // REV 2 CTRE 20
			kFrontRightDrivePort = 10, // REV 10 CTRE 10
			kFrontRightTurningPort = 11, // REV 11 CTRE 11
			kFrontRightAbsEncoderPort = 21, // REV 0 CTRE 21
			kBackLeftDrivePort = 14, // REV 14 CTRE 14
			kBackLeftTurningPort = 15, // REV 15 CTRE 15
			kBackLeftAbsEncoderPort = 23, // REV 3 CTRE 23
			kBackRightDrivePort = 12, // REV 12 CTRE 12
			kBackRightTurningPort = 13, // REV 13 CTRE 13
			kBackRightAbsEncoderPort = 24, // REV 1 CTRE 24
			kMaxDriveCurrent = 80, kMaxTurnCurrent = 80;
	public static boolean kFrontLeftDriveReversed = true,
			kFrontLeftTurningReversed = true, kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
			kFrontRightAbsEncoderReversed = false, kBackLeftDriveReversed = false,
			kBackLeftTurningReversed = true, kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = false, kBackRightTurningReversed = true,
			kBackRightAbsEncoderReversed = false, kGyroReversed = true;
	public static ModuleLimits moduleLimitsFree = new ModuleLimits(
			DriveConstants.kMaxSpeedMetersPerSecond,
			maxTranslationalAcceleration.get(), maxRotationalAcceleration.get());

	public static class TrainConstants {
		/**
		 * Which swerve module it is (SWERVE EXCLUSIVE)
		 */
		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(
				VecBuilder.fill(0.003, 0.003, 0.0002));
		public static double kWheelDiameter = Units.inchesToMeters(3.873),
				kDriveMotorGearRatio = 6.75, kTurningMotorGearRatio = 150 / 7,
				kT = 1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp, kDeadband = 0.05,
				weight = Units.lbsToKilograms(110);
		public static final MotorConstantContainer pathplannerTranslationConstantContainer = new MotorConstantContainer(
				0.001, 0.001, 0.001, 5, 2.4, 0.006),
				pathplannerRotationConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0.006),
				//rev 
				overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0.001), //Average the turning motors for these vals.
				//ctre
				/*overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 50, 0, .1), //Average the turning motors for these vals.	*/
				overallDriveMotorConstantContainer = new MotorConstantContainer(.1,
						.13, 0.001, 0.05, 0, 0.000);
	}

	public static RobotProfile mainRobotProfile = new RobotProfile(
			kMaxSpeedMetersPerSecond, maxTranslationalAcceleration.get(),
			kMaxTurningSpeedRadPerSec, TrainConstants.weight, kBumperToBumperWidth,
			kBumperToBumperLength);

	public static final class RobotPhysicsSimulationConfigs {
		public static final int SIM_ITERATIONS_PER_ROBOT_PERIOD = 5;
		/* Swerve Module Simulation */
		public static final double DRIVE_MOTOR_FREE_FINAL_SPEED_RPM = 859;
		public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1),
				STEER_MOTOR = DCMotor.getKrakenX60Foc(1);
		public static final double DRIVE_WHEEL_ROTTER_INERTIA = 0.05;
		public static final double STEER_INERTIA = 0.015;
		public static final double FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ = 10;
		public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = Math
				.toRadians(1200);
		public static final double TIME_CHASSIS_STOPS_ROTATING_NO_POWER_SEC = 0.3;
		public static final double DEFAULT_ROBOT_MASS = 110;
		public static final double DEFAULT_BUMPER_WIDTH_METERS = Units
				.inchesToMeters(35.5);
		public static final double DEFAULT_BUMPER_LENGTH_METERS = Units
				.inchesToMeters(35.5);
		/* https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction */
		public static final double ROBOT_BUMPER_COEFFICIENT_OF_FRICTION = 0.85;
		/* https://en.wikipedia.org/wiki/Coefficient_of_restitution */
		public static final double ROBOT_BUMPER_COEFFICIENT_OF_RESTITUTION = 0.05;
		/* Gyro Sim */
		public static final double GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ = 100;
		public static final double SKIDDING_AMOUNT_AT_THRESHOLD_RAD = Math
				.toRadians(1.2);
		/*
		* https://store.ctr-electronics.com/pigeon-2/
		* for a well-installed one with vibration reduction, only 0.4 degree
		* but most teams just install it directly on the rigid chassis frame (including my team :D)
		* so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
		* which is the average velocity during normal swerve-circular-offense
		* */
		public static final double NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD = Math
				.toRadians(1.2);
		public static final double AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST = Math
				.toRadians(60);
	}
}
