package frc.robot.utils.drive;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation.DriveTrainSimulationProfile;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation.WHEEL_GRIP;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
public class DriveConstants {
	// YEARLYUPDATE:  Change these to the drivetrain being used. Duh -N
	// If true, tank/mecanum use their native PIDs. If false, tank/mech output their
	// voltages directly
	public static final boolean enablePID = true;
	public static final MotorVendor robotMotorController = MotorVendor.CTRE_ON_CANIVORE;
	public static final CANBus driveCanBus = new CANBus("drivetrain"); // Leave null if CTRE_ON_RIO
	public static final DriveTrainType driveType = DriveTrainType.SWERVE;
	// This one is swerve-exclusive
	public static final SwerveModuleType swerveModuleType = SwerveModuleType.THRIFTYSWERVE;
	public static final GyroType gyroType = GyroType.PIGEON;
	public static final boolean useThriftyEncoder = false;
	public static final WHEEL_GRIP gripType = WHEEL_GRIP.VEX_GRIP_V2;
	public static final int maximumAutoCycles = 6;
	public class DriverConstants {
		public static final double kDeadband = 0.1, translationalResponseCurveExponent = 2.4,
				rotationalResponseCurveExponent = 2.4;
	}

	public static DCMotor getDriveTrainMotors(int number) {
		switch (robotMotorController) {
			case NEO_SPARK_MAX:
				return DCMotor.getNEO(number);

			case VORTEX_SPARK_FLEX:
				return DCMotor.getNeoVortex(number);

			// These cases assume that drivetrain uses kraken x60s FOC, because Grant bought
			// 20 of those.
			// I had to sell my left kidney for those krakens-N
			case CTRE_ON_CANIVORE:
			case CTRE_ON_RIO:
				return DCMotor.getKrakenX60Foc(number);
			default:
				// returns completely defunct motor
				return new DCMotor(0, 0, 0, 0, 0, 0);
		}

	}
	
	public static DCMotor getDriveTrainMotors(int number, double reduction) {
		switch (robotMotorController) {
			case NEO_SPARK_MAX:
				return DCMotor.getNEO(number).withReduction(reduction);

			case VORTEX_SPARK_FLEX:
				return DCMotor.getNeoVortex(number).withReduction(reduction);

			// These cases assume that drivetrain uses kraken x60s FOC, because Grant bought
			// 20 of those.
			// I had to sell my left kidney for those krakens-N
			case CTRE_ON_CANIVORE:
			case CTRE_ON_RIO:
				return DCMotor.getKrakenX60Foc(number).withReduction(reduction);
			default:
				// returns completely defunct motor
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

	public enum SwerveModuleType {
		SDSMK4I,
		THRIFTYSWERVE,
		SHIFTING_THIFTYSWERVE
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
			"Drive/MaxTranslationalAcceleration", 30,TuningConstants.isTuningMacros);
	public static final LoggableTunedNumber maxRotationalAcceleration = new LoggableTunedNumber(
			"Drive/MaxRotationalAcceleration", 2 * Math.PI * 50,TuningConstants.isTuningMacros);
	public static boolean fieldOriented = true;
	public static boolean autoAvoidance = false;
	// 135-Blocks was tested on a chassis with all CANSparkMaxes, as well as all
	// Kraken-x60s.
	public static final double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
			kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
			kBumperToBumperWidth = Units.inchesToMeters(37.5), // Distance between bumpers
			kBumperToBumperLength = Units.inchesToMeters(37.5), // Distance between bumpers
			kDriveBaseRadius = Math.sqrt(
					kChassisLength * kChassisLength + kChassisWidth * kChassisWidth)
					/ 2,
			// Distance from center of robot to the farthest module
			// To find these set them to zero, then turn the robot on and manually set the
			// wheels straight.
			// The encoder values being read are then your new Offset values
			// REV Offsets
			/*
			 * kFrontLeftAbsEncoderOffsetRad = 0.562867,
			 * kFrontRightAbsEncoderOffsetRad = 0.548137,
			 * kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
			 * kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861,
			 */
			// Ctre Offsets
			kFrontLeftAbsEncoderOffsetRad = 0, kFrontRightAbsEncoderOffsetRad = 0, // -.935 , BR -.7792
			kBackLeftAbsEncoderOffsetRad = 0, kBackRightAbsEncoderOffsetRad = 0, // -.2392 FL -.5813
			SKID_THRESHOLD = .5, // Meters per second
			TURN_DEADBAND_AMPS = 10, //minimum amperage allowed on turn motors (to prevent weirdo noises/eating voltage)
			MAX_G = 1.5;
	public static double kMaxSpeedMetersPerSecond = 6.0, // 15.1
			kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI; // 1.33655 *2 *Math.PI
	public static PathConstraints pathConstraints = new PathConstraints(
			6, 17.5,
			kMaxTurningSpeedRadPerSec, maxRotationalAcceleration.get());
	// kP = 0.1, kI = 0, kD = 0, kDistanceMultipler = .2; //for autoLock
	// Declare the position of each module
	public static final Translation2d[] kModuleTranslations = {
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2)
	};
	public static final int kFrontLeftDrivePort = 10, // REV 16 CTRE 16
			kFrontLeftTurningPort = 11, // REV 16 CTRE 17
			kFrontLeftAbsEncoderPort = 12, // REV 2 CTRE 20
			kFrontRightDrivePort = 13, // REV 10 CTRE 10
			kFrontRightTurningPort = 14, // REV 11 CTRE 11
			kFrontRightAbsEncoderPort = 15, // REV 0 CTRE 21
			kBackLeftDrivePort = 19, // REV 14 CTRE 14
			kBackLeftTurningPort = 20, // REV 15 CTRE 15
			kBackLeftAbsEncoderPort = 21, // REV 3 CTRE 23
			kBackRightDrivePort = 16, // REV 12 CTRE 12
			kBackRightTurningPort = 17, // REV 13 CTRE 13
			kBackRightAbsEncoderPort = 18, // REV 1 CTRE 24
			kGyroPort = 9, // REV DOESN'T MATTER, USE kUSB1 CTRE 18
			kFrontLeftShifterForward = 0, kFrontLeftShifterReverse = 1,
			kFrontRightShifterForward = 2, kFrontRightShifterReverse = 3,
			kBackLeftShifterForward = 4, kBackLeftShifterReverse = 5,
			kBackRightShifterForward = 6, kBackRightShifterReverse = 7,
			kMaxDriveCurrent = 65, kMaxTurnCurrent = 20;
	public static final boolean kFrontLeftDriveReversed = false,
			kFrontLeftTurningReversed = false, kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = false,
			kFrontRightAbsEncoderReversed = false, kBackLeftDriveReversed = true,
			kBackLeftTurningReversed = false, kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = true, kBackRightTurningReversed = false,
			kBackRightAbsEncoderReversed = false;
	public static ModuleLimits moduleLimitsLow = new ModuleLimits(
			DriveConstants.kMaxSpeedMetersPerSecond,
			maxTranslationalAcceleration.get(), DriveConstants.kMaxTurningSpeedRadPerSec);
	public static ModuleLimits moduleLimitsHigh = new ModuleLimits(
			getDriveTrainMotors(1).freeSpeedRadPerSec / TrainConstants.kDriveMotorGearRatioHigh
					* TrainConstants.kWheelDiameter.get() / 2,
			Math.min(
					getDriveTrainMotors(1).getTorque(getDriveTrainMotors(1).stallCurrentAmps)
							* TrainConstants.kDriveMotorGearRatioHigh / (TrainConstants.kWheelDiameter.get() / 2),
					9.8 * TrainConstants.weight / 4 * WHEEL_GRIP.VEX_GRIP_V2.cof) * 4 / TrainConstants.weight,
			getDriveTrainMotors(1).freeSpeedRadPerSec / TrainConstants.kDriveMotorGearRatioHigh
					* TrainConstants.kWheelDiameter.get() / 2
					/ new Translation2d(kChassisLength / 2, kChassisWidth / 2).getNorm());
	public static class AutopilotConstants {
			public static final APConstraints kTightAutopilotAPConstraints =
			new APConstraints().withAcceleration(maxTranslationalAcceleration.get()/2).withJerk(1.5);

			public static final APProfile kTightProfile =
			new APProfile(kTightAutopilotAPConstraints)
				.withErrorXY(Centimeters.of(1))
				.withErrorTheta(Degrees.of(1))
				.withBeelineRadius(Centimeters.of(10));
			public static final APConstraints kFastAPConstraints =
			new APConstraints().withAcceleration(maxTranslationalAcceleration.get()*2).withJerk(maxTranslationalAcceleration.get()*2);

			public static final APProfile kFastProfile =
			new APProfile(kFastAPConstraints)
				.withErrorXY(Centimeters.of(5))
				.withErrorTheta(Degrees.of(5))
				.withBeelineRadius(Centimeters.of(50));
	}
	public static class TrainConstants {

		/**
		 * Which swerve module it is (SWERVE EXCLUSIVE)
		 */
		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		// Mecanum exclusive, shows the initial offset of the wheel
		public static final double mecanumInitialAngleOffsetDegrees = 135, odomHz =250;
		public static final Rotation2d robotOffsetAngleDirection = Rotation2d.fromDegrees(-90); // 90 degrees makes robot
																								// front = facing left,
																								// 270 = right
		public static final Matrix<N3, N1> odometryStateStdDevs = new Matrix<>(
				VecBuilder.fill(0.003, 0.003, 0.002));
		public static final LoggableTunedNumber kWheelDiameter = new LoggableTunedNumber("Drive/moduleDiameter", .1016, TuningConstants.isTuningModules),
				RPMMatch = new LoggableTunedNumber("Drive/Module/RPMMatch", 4000,TuningConstants.isTuningModules),
				extendTime = new LoggableTunedNumber("Drive/Module/extendTime", 200,TuningConstants.isTuningModules);
		public static final double kMaxAngularSpeedRadiansPerSecond = 2 * DriveConstants.kMaxSpeedMetersPerSecond
				/ (kWheelDiameter.get()),
				kDriveMotorGearRatioLow = 5.14, kDriveMotorGearRatioHigh = 3, kTurningMotorGearRatio = 25,
				kT = 1.0 / getDriveTrainMotors(1).KtNMPerAmp,
				moi = 2.8732, // kg m^2, moment of inertia of the robot
				weight = Units.lbsToKilograms(56); // test chassis
		public static final MotorConstantContainer pathplannerTranslationConstantContainer = new MotorConstantContainer(
				0.001, 0.001, 0.001, .675,.125, 0),
				pathplannerRotationConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0);

	}

	public static ModuleConfig mainModuleConfig;
	public static RobotConfig mainConfig;
	public static PathFollowingController mainController;
	public static MotorConstantContainer overallTurningMotorConstantContainer, overallDriveMotorConstantContainer;
	static {
		if (driveType == DriveTrainType.TANK) {

			mainModuleConfig = new ModuleConfig(TrainConstants.kWheelDiameter.get() / 2, kMaxSpeedMetersPerSecond, 1.25,
					getDriveTrainMotors(2, TrainConstants.kDriveMotorGearRatioLow), kMaxDriveCurrent, 2);
			mainConfig = new RobotConfig(TrainConstants.weight, TrainConstants.moi, mainModuleConfig, kChassisWidth);
			mainController = new PPLTVController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0),
					.02, kMaxSpeedMetersPerSecond);
		} else {
			mainModuleConfig = new ModuleConfig(TrainConstants.kWheelDiameter.get() / 2, kMaxSpeedMetersPerSecond, 1.25,
					getDriveTrainMotors(1, TrainConstants.kDriveMotorGearRatioLow), kMaxDriveCurrent, 1);
			mainConfig = new RobotConfig(TrainConstants.weight, TrainConstants.moi, mainModuleConfig, kModuleTranslations);
			mainController = new PPHolonomicDriveController(
					new PIDConstants(TrainConstants.pathplannerTranslationConstantContainer.getP(),
							TrainConstants.pathplannerTranslationConstantContainer.getI(),
							TrainConstants.pathplannerTranslationConstantContainer.getD()),
					new PIDConstants(TrainConstants.pathplannerRotationConstantContainer.getP(),
							TrainConstants.pathplannerRotationConstantContainer.getI(),
							TrainConstants.pathplannerRotationConstantContainer.getD()));
		}
		if (Constants.currentMode == Mode.SIM) {
			overallTurningMotorConstantContainer = new MotorConstantContainer(
					0.02, 0.001, 0.001, 12, 0.01, 0.001);
			overallDriveMotorConstantContainer = new MotorConstantContainer(.1, 
					.13, 0.001, .05, 0, 0);
		} else {
			if (robotMotorController == MotorVendor.CTRE_ON_CANIVORE
					|| robotMotorController == MotorVendor.CTRE_ON_RIO) {
				overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.25, 0.04, 0.001, 1000, 0, 50); // Average the turning motors for these vals.
						//Test chassis: 1.65, 125, 0.6, 200, 35, 13.25
				overallDriveMotorConstantContainer = new MotorConstantContainer(5, 
						.09, 0.001, 35, 0.0, 0.00);
			} else {
				overallTurningMotorConstantContainer = new MotorConstantContainer(
						0.001, 0.001, 0.001, 5, 0, 0.001); // Average the turning motors for these vals.
				overallDriveMotorConstantContainer = new MotorConstantContainer(2, 
						.1, 0.001, 5, 0.25, 0.05);
			}
		}
	}
	public static DriveTrainSimulationProfile mainRobotProfile = new DriveTrainSimulationProfile(
			kMaxSpeedMetersPerSecond, maxTranslationalAcceleration.get(),
			kMaxTurningSpeedRadPerSec, maxRotationalAcceleration.get(), TrainConstants.weight, kBumperToBumperWidth,
			kBumperToBumperLength);

	public static final class RobotPhysicsSimulationConfigs {
		public static final int SIM_ITERATIONS_PER_ROBOT_PERIOD = 5;
		public static final double SIMULATION_DT = Robot.defaultPeriodSecs / SIM_ITERATIONS_PER_ROBOT_PERIOD;
		public static final double MAX_FAKE_G = 0.1;
	}
}
