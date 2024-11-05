package frc.robot.utils.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation.DriveTrainSimulationProfile;

public class DriveConstants {
	//If true, tank/mecanum use their native PIDs. If false, tank/mech output their voltages directly
	public static final boolean enablePID = true;
	public static final MotorVendor robotMotorController = MotorVendor.CTRE_ON_CANIVORE;
	public static final DriveTrainType driveType = DriveTrainType.SWERVE;
	public static final GyroType gyroType = GyroType.PIGEON;
	public static DCMotor getDriveTrainMotors(int number){
		switch (robotMotorController) {
			case NEO_SPARK_MAX:
				return DCMotor.getNEO(number);

			case VORTEX_SPARK_FLEX:
				return DCMotor.getNeoVortex(number);
			
			//These cases assume that drivetrain uses kraken x60s FOC, because Grant bought 20 of those. 
			//I had to sell my left kidney for those krakens-N
			case CTRE_ON_CANIVORE:
				case CTRE_ON_RIO:
					return DCMotor.getKrakenX60Foc(number);
			default:
				//returns completely defunct motor
				return new DCMotor(0, 0, 0, 0, 0, 0);
		}

	}
	public static DCMotor getDriveTrainMotors(int number, double reduction){
		switch (robotMotorController) {
			case NEO_SPARK_MAX:
				return DCMotor.getNEO(number).withReduction(reduction);

			case VORTEX_SPARK_FLEX:
				return DCMotor.getNeoVortex(number).withReduction(reduction);
			
			//These cases assume that drivetrain uses kraken x60s FOC, because Grant bought 20 of those. 
			//I had to sell my left kidney for those krakens-N
			case CTRE_ON_CANIVORE:
				case CTRE_ON_RIO:
					return DCMotor.getKrakenX60Foc(number).withReduction(reduction);
			default:
				//returns completely defunct motor
				return new DCMotor(0, 0, 0, 0, 0, 0).withReduction(reduction);
		}

	}
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
			"Drive/MaxTranslationalAcceleration", 22);
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
			MAX_G = 0.5;
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
	public static final int kFrontLeftDrivePort = 16, // REV 16 CTRE 16
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
			kMaxDriveCurrent = 180, kMaxTurnCurrent = 80;
	public static final boolean kFrontLeftDriveReversed = true,
			kFrontLeftTurningReversed = true, kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
			kFrontRightAbsEncoderReversed = false, kBackLeftDriveReversed = false,
			kBackLeftTurningReversed = true, kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = false, kBackRightTurningReversed = true,
			kBackRightAbsEncoderReversed = false;
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
		//Mecanum exclusive, shows the initial offset of the wheel
		public static final double mecanumInitialAngleOffsetDegrees = 135, odomHz = 250;
		public static final Rotation2d robotOffsetAngleDirection = Rotation2d.fromDegrees(0); //90 degrees makes robot front = facing left, 270 = right
		public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(
				VecBuilder.fill(0.003, 0.003, 0.0002));
		public static double kDeadband = 0.05;
		public static final double kWheelDiameter = Units.inchesToMeters(4),
				kDriveMotorGearRatio = 6.75, kTurningMotorGearRatio = 150 / 7,
				kT = 1.0 / getDriveTrainMotors(1).KtNMPerAmp,
				weight = Units.lbsToKilograms(40); //test chassis
		public static final MotorConstantContainer pathplannerTranslationConstantContainer = new MotorConstantContainer(
				0.001, 0.001, 0.001, 5, 0, 0),
				pathplannerRotationConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0),
				//sim
				overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.02, 0.001, 0.001, 12, 0.01, 0.001),
				/*//rev 
				overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0.001), //Average the turning motors for these vals.
				//ctre
				//overallTurningMotorConstantContainer = new MotorConstantContainer(
				//		0.001, 0.001, 0.001, 50, 0, .1), //Average the turning motors for these vals.	*/
				overallDriveMotorConstantContainer = new MotorConstantContainer(.1,
						.13, 0.001, 0.05, 0, 0.000);
	}
	public static ModuleConfig mainModuleConfig;
	public static RobotConfig mainConfig;
	public static PathFollowingController mainController;
	static{
		if (driveType == DriveTrainType.TANK){

			mainModuleConfig = new ModuleConfig(TrainConstants.kWheelDiameter/2, kMaxSpeedMetersPerSecond, 1.25, getDriveTrainMotors(2,TrainConstants.kDriveMotorGearRatio), kMaxDriveCurrent, 2);
			mainConfig = new RobotConfig(TrainConstants.weight, 2.887, mainModuleConfig, kChassisWidth);
			mainController = new PPLTVController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), kBumperToBumperWidth, MAX_G);
		}else{
		 	mainModuleConfig = new ModuleConfig(TrainConstants.kWheelDiameter/2, kMaxSpeedMetersPerSecond, 1.25, getDriveTrainMotors(1,TrainConstants.kDriveMotorGearRatio), kMaxDriveCurrent, 1);
			mainConfig = new RobotConfig(TrainConstants.weight, 2.887, mainModuleConfig, kChassisWidth,kChassisLength);
			mainController = new PPHolonomicDriveController(new PIDConstants(TrainConstants.pathplannerTranslationConstantContainer.getP(), TrainConstants.pathplannerTranslationConstantContainer.getI(), TrainConstants.pathplannerTranslationConstantContainer.getD()), new PIDConstants(TrainConstants.pathplannerRotationConstantContainer.getP(), TrainConstants.pathplannerRotationConstantContainer.getI(), TrainConstants.pathplannerRotationConstantContainer.getD()));
		}
	}
	public static DriveTrainSimulationProfile mainRobotProfile = new DriveTrainSimulationProfile(
			kMaxSpeedMetersPerSecond, maxTranslationalAcceleration.get(),
			kMaxTurningSpeedRadPerSec, maxRotationalAcceleration.get(),TrainConstants.weight, kBumperToBumperWidth,
			kBumperToBumperLength);

		public static final class RobotPhysicsSimulationConfigs {
		public static final int SIM_ITERATIONS_PER_ROBOT_PERIOD = 5;
		public static final double SIMULATION_DT = Robot.defaultPeriodSecs / SIM_ITERATIONS_PER_ROBOT_PERIOD;
		public static final double MAX_FAKE_G = 0.1;
	}
}
