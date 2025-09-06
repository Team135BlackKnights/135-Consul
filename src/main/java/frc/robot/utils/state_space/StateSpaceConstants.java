package frc.robot.utils.state_space;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.EncoderType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.MotorConstantContainer;


import com.ctre.phoenix6.CANBus;

public class StateSpaceConstants {
	public static boolean debug = true;
	public class Controls {
		/* Enter any non-button controls here.
		 * Left trigger is used for RPM speed. 0-0 1-7100.
		 * Right stick is used for arm Speed 0-0 1-maxArmSpeed.
		 * Left stick is used for elevator Speed 0-0 1-maxElevatorSpeed.
		 */
		public static double kDeadband = 0.1, kArmDeadband = 0.1,
				armMoveSpeed = .01, elevatorMoveSpeed = 1;
		public static JoystickButton goto4000Button = new JoystickButton(
				RobotContainer.manipController, 5), //left bumper
				go45Button = new JoystickButton(RobotContainer.manipController, 1), //a
				go0Button = new JoystickButton(RobotContainer.manipController, 2), //b
				go2ftButton = new JoystickButton(RobotContainer.manipController, 3), //x
				go0ftButton = new JoystickButton(RobotContainer.manipController, 4); //y
		public static POVButton gotoUpRight = new POVButton(
				RobotContainer.manipController, 45), //upRight
				gotoUpLeft = new POVButton(RobotContainer.manipController, 315);
	}

	public class Flywheel {
		public static CANBus CANBus = new CANBus();
		public static MotorVendor motorVendor = MotorVendor.CTRE_ON_RIO;
		public static EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER;
		public static boolean inverted = false;
		public static boolean isEncoderInverted = false;
		public static boolean isBrake = false;
		//Encoder ID and CANBus only matter for CTRE
		public static int kMotorID = 20, kEncoderID = 25, maxRPM = 8700, currentLimit = 20;
		public static MotorConstantContainer flywheelValueHolder = new MotorConstantContainer(
				-0.089838, 0.0015425 * .88, 0.0039717 * 1, 0,0, 0);
		public static double m_KalmanModel = 3, m_KalmanEncoder = 0.01,
				m_LQRQelms = 1, m_LQRRVolts = 12, flywheelGearing = 1.5,
				encoderGearing = 1,
				encoderOffsetRotations = 0,
				MOI = 0.001;
	}

	public class DoubleJointedArm {
		public static CANBus CANBus = new CANBus();
		public static boolean armInverted = false;
		public static boolean isArmEncoderInverted = false;
		public static boolean isElbowEncoderInverted = false;
		public static boolean elbowInverted = false;
		public static EncoderType doubleJointedEncoderType = EncoderType.NO_ATTACHED_ENCODER;
		public static boolean isBrake = false;
		public static int kArmMotorID = 30, kElbowMotorID = 31, kArmEncoderID = 28,  kElbowEncoderID = 29;
		public static double[] macroTopLeft = { -1.5, 1, 0
		}, macroTopRight = { 1.5, 1, 1
		}; //0 = false, 1 = true for the last value 
		public static double armCurrentLimit = 60, armGearing = 70,
				armMinRad = Double.NEGATIVE_INFINITY,
				armMaxRad = Double.POSITIVE_INFINITY,
				elbowMinRad = Double.NEGATIVE_INFINITY,
				elbowMaxRad = Double.POSITIVE_INFINITY, elbowCurrentLimit = 60,
				elbowGearing = 45, armLength = Units.inchesToMeters(46.25),
				armEncoderGearing = 1,
				armEncoderOffsetRotations = 0,
				elbowEncoderGearing = 1,
				elbowEncoderOffsetRotations = 0,
				elbowLength = Units.inchesToMeters(41.8),
				simSizeWidth = (armLength + elbowLength) * 2,
				simSizeLength = (armLength + elbowLength) * 2,
				physicalX = simSizeWidth / 2, physicalY = simSizeLength / 2;
		//qelms and relms
		public static LoggableTunedNumber qPos = new LoggableTunedNumber("DoubleJointedArmS/qPos",0.01745,Constants.TuningConstants.isTuningDoubleJointedArm),
		qVel = new LoggableTunedNumber("DoubleJointedArmS/qVel",0.1745329,Constants.TuningConstants.isTuningDoubleJointedArm),
		qError = new LoggableTunedNumber("DoubleJointedArmS/qError",10,Constants.TuningConstants.isTuningDoubleJointedArm),
		rPos = new LoggableTunedNumber("DoubleJointedArmS/rPos",.01745/4,Constants.TuningConstants.isTuningDoubleJointedArm);
	}

	public class SingleJointedArm {
		public static CANBus CANBus = new CANBus();
		public static MotorVendor motorVendor = MotorVendor.CTRE_ON_RIO;
		public static EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER;
		public static boolean inverted = false;
		public static boolean isEncoderInverted = false;
		public static boolean isBrake = false;
		public static int kMotorID = 30, kEncoderID = 26;
		public static MotorConstantContainer armValueHolder = new MotorConstantContainer(
				.001, .001, .001, 0,0, 0); //must have position set in SysId
		public static double m_KalmanModelPosition = .015,
				statorCurrentLimit = 150, m_KalmanModelVelocity = .17,
				m_KalmanEncoderPosition = 0.003, //in rads
				m_KalmanEncoderVelocity = 0.003, //in rad per sec
				m_LQRQelmsPosition = Units.degreesToRadians(1),
				m_LQRQelmsVelocity = Units.degreesToRadians(45.0), m_LQRRVolts = 12,
				armGearing = 200,
				encoderGearing = 1,
				encoderOffsetRotations = 0,
				maxSpeed = DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec,
				maxAcceleration = DCMotor.getKrakenX60Foc(1).freeSpeedRadPerSec / 2,
				startingPosition = Units.degreesToRadians(-36),
				maxPosition = Units.degreesToRadians(45),
				armLength = Units.inchesToMeters(15),
				armMass = Units.lbsToKilograms(14),
				physicalX = Units.inchesToMeters(20),
				physicalY = Units.inchesToMeters(DriveConstants.kChassisWidth / 2),
				simX = Units.inchesToMeters(11.5), simY = Units.inchesToMeters(0),
				simZ = Units.inchesToMeters(4.82);
		public static int currentLimit = 60;
	}

	public class Elevator {
		public static CANBus CANBus = new CANBus();
		public static MotorVendor motorVendor = MotorVendor.CTRE_ON_RIO;
		public static EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER;
		public static boolean inverted = false;
		public static boolean isEncoderInverted = false;
		public static boolean isBrake = false;
		public static int kMotorID = 40,  kEncoderID = 27, currentLimit = 60;
		public static MotorConstantContainer elevatorValueHolder = new MotorConstantContainer(
				.001, .001, .001, 0,0, 0); //must have position set in SysId
		public static double m_KalmanModelPosition = Units.inchesToMeters(1),
				m_KalmanModelVelocity = Units.inchesToMeters(40),
				m_KalmanEncoderPosition = 0.001, m_KalmanEncoderVelocity = 0.001, m_LQRQelmsPosition = 1,
				m_LQRQelmsVelocity = 10, m_LQRRVolts = 12, elevatorGearing = 1.5,
				encoderGearing = 1,
				encoderOffsetRotations = 0,
				carriageMass = Units.lbsToKilograms(10),
				drumRadius = Units.inchesToMeters(.75),
				maxSpeed = Units.feetToMeters(6),
				maxAcceleration = Units.feetToMeters(6),
				startingPosition = Units.inchesToMeters(0),
				maxPosition = Units.feetToMeters(3),
				armLength = Units.inchesToMeters(5),
				physicalX = Units.inchesToMeters(20),
				physicalY = Units.inchesToMeters(DriveConstants.kChassisWidth / 2),
				simX = Units.inchesToMeters(5), simY = Units.inchesToMeters(5),
				simZ = Units.inchesToMeters(5);
	}
}
