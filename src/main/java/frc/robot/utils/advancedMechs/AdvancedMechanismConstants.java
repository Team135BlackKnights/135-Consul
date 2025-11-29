package frc.robot.utils.advancedMechs;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Constants.EncoderType;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.drive.DriveConstants.MotorVendor;

public class AdvancedMechanismConstants {
    public class PinkArm {
        public static String CANBus = "everything";
        public static MotorVendor motorVendor = MotorVendor.CTRE_ON_CANIVORE;

        public class Shoulder {
            public static boolean invertedFL = false, invertedFR = false, invertedBL = false, invertedBR = false;
            public static boolean isEncoderInverted = false;
            public static boolean isBrake = true;
            public static int kMotorFLID = 20, kMotorFRID = 21, kMotorBLID = 22, kMotorBRID = 23;
            public static EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER; // only other is CTRE
            public static double statorCurrentLimit = 150,
                    shoulderGearing = 100,
                    encoderGearing = 1,
                    encoderOffsetRotations = .0,
                    shoulderMOI = 0.0925974241,
                    startingPosition = 0,
                    maxPosition = Units.degreesToRadians(135),
                    shoulderLength = Units.inchesToMeters(15),
                    shoulderMass = Units.lbsToKilograms(14),
                    physicalX = Units.inchesToMeters(3.5),
                    physicalY = Units.inchesToMeters(.5),
                    physicalZ = Units.inchesToMeters(13.4);
            public static DCMotor gearbox = DCMotor.getKrakenX60(1);
            public static LoggableTunedNumber kP = new LoggableTunedNumber("PinkArm/Shoulder/kP", 2.5,
                    TuningConstants.isTuningArm),
                    kI = new LoggableTunedNumber("PinkArm/Shoulder/kI", 0.000, TuningConstants.isTuningArm),
                    kD = new LoggableTunedNumber("PinkArm/Shoulder/kD", 0.02, TuningConstants.isTuningArm),
                    kS = new LoggableTunedNumber("PinkArm/Shoulder/kS", .3, TuningConstants.isTuningArm),
                    kG = new LoggableTunedNumber("PinkArm/Shoulder/kG", 0, TuningConstants.isTuningArm),
                    kV = new LoggableTunedNumber("PinkArm/Shoulder/kV", 0.3, TuningConstants.isTuningArm),
                    maxSpeed = new LoggableTunedNumber("PinkArm/Shoulder/maxSpeed", 200, TuningConstants.isTuningArm),
                    maxAcceleration = new LoggableTunedNumber("PinkArm/Shoulder/maxAccel", 20,
                            TuningConstants.isTuningArm);
        }
    }

    public class AlgaeArm {
        public static String CANBus = "everything";
        public static MotorVendor motorVendor = MotorVendor.CTRE_ON_CANIVORE;
        public static EncoderType encoderType = EncoderType.DUTY_CYCLE;
        public static boolean inverted = false;
        public static boolean isEncoderInverted = false;
        public static boolean isBrake = true;
        public static int kMotorID = 32, kEncoderID = 0;// rio port
        public static DCMotor gearbox = DCMotor.getKrakenX60(1);
        public static MotorConstantContainer armValueHolder = new MotorConstantContainer(
                .001, .001, .001, 0, 0, 0); // must have position set in SysId
        public static double statorCurrentLimit = 150,
                armGearing = 3,
                encoderGearing = 1,
                encoderOffsetRotations = .171,
                armMOI = 0.0925974241,
                startingPosition = 0,

                maxPosition = Units.degreesToRadians(165),
                armLength = Units.inchesToMeters(15),
                armMass = Units.lbsToKilograms(14),
                physicalX = Units.inchesToMeters(3.5),
                physicalY = Units.inchesToMeters(.5),
                physicalZ = Units.inchesToMeters(13.4);
        public static LoggableTunedNumber m_KalmanModelPosition = new LoggableTunedNumber(
                "AlgaeArm/SS/KalmanModelPosition", Units.degreesToRadians(15), TuningConstants.isTuningArm),
                m_KalmanModelVelocity = new LoggableTunedNumber("AlgaeArm/SS/KalmanModelVelocity",
                        Units.degreesToRadians(20), TuningConstants.isTuningArm),
                m_KalmanEncoderPosition = new LoggableTunedNumber("AlgaeArm/SS/KalmanEncoderPosition",
                        0.0004, TuningConstants.isTuningArm),
                m_KalmanEncoderVelocity = new LoggableTunedNumber("AlgaeArm/SS/KalmanEncoderVelocity",
                        0.0005, TuningConstants.isTuningArm),
                m_LQRQelmsPosition = new LoggableTunedNumber("AlgaeArm/SS/LQRQelmsPosition",
                        .01, TuningConstants.isTuningArm),
                m_LQRQelmsVelocity = new LoggableTunedNumber("AlgaeArm/SS/LQRQelmsVelocity",
                        .75, TuningConstants.isTuningArm),
                m_LQRRVolts = new LoggableTunedNumber("AlgaeArm/SS/LQRRVolts", 12, TuningConstants.isTuningArm),
                // for FF Model
                kS = new LoggableTunedNumber("AlgaeArm/FF/kS", .3, TuningConstants.isTuningArm),
                kG = new LoggableTunedNumber("AlgaeArm/FF/kG", 0, TuningConstants.isTuningArm), // tune in real
                kV = new LoggableTunedNumber("AlgaeArm/FF/kV", 0.3, TuningConstants.isTuningArm),
                kP = new LoggableTunedNumber("AlgaeArm/kP", 2.5, TuningConstants.isTuningArm),
                kI = new LoggableTunedNumber("AlgaeArm/kI", 0.000, TuningConstants.isTuningArm),
                kD = new LoggableTunedNumber("AlgaeArm/kD", 0.02, TuningConstants.isTuningArm),
                maxSpeed = new LoggableTunedNumber("AlgaeArm/maxSpeed", 200, TuningConstants.isTuningArm),
                maxAcceleration = new LoggableTunedNumber("AlgaeArm/maxAccel", 20, TuningConstants.isTuningArm);
        public static int currentLimit = 160;
    }

    public class DynamicElevator {

        public static final LoggableTunedNumber kP = new LoggableTunedNumber("Elevator/Elevator kP", 850,
                TuningConstants.isTuningElevator), // 450 in sim
                kI = new LoggableTunedNumber("Elevator/Elevator kI", 0.000, TuningConstants.isTuningElevator),
                kD = new LoggableTunedNumber("Elevator/Elevator kD", 65, TuningConstants.isTuningElevator), // 30 in sim
                kS = new LoggableTunedNumber("Elevator/Elevator kS", 16, TuningConstants.isTuningElevator), // .9 in sim
                kG = new LoggableTunedNumber("Elevator/Elevator kG", 0, TuningConstants.isTuningElevator), // .065 in
                                                                                                           // sim
                kV = new LoggableTunedNumber("Elevator/Elevator kV", 1, TuningConstants.isTuningElevator), // 11.8 in
                                                                                                           // sim
                kA = new LoggableTunedNumber("Elevator/Elevator kA", 0.0, TuningConstants.isTuningElevator), // .18 in
                                                                                                             // ism
                maxAccel = new LoggableTunedNumber("Elevator/Elevator Max Acceleration", 5.5,
                        TuningConstants.isTuningElevator), // 1 in sim
                maxVelocity = new LoggableTunedNumber("Elevator/Elevator Max Velocity", 7,
                        TuningConstants.isTuningElevator), // 4.5 in sim
                distanceX = new LoggableTunedNumber("Elevator/distanceX", 14, TuningConstants.isTuningElevator),
                distanceY = new LoggableTunedNumber("Elevator/distanceY", 8, TuningConstants.isTuningElevator),
                widthX = new LoggableTunedNumber("Elevator/widthX", 4, TuningConstants.isTuningElevator),
                widthY = new LoggableTunedNumber("Elevator/widthY", 6, TuningConstants.isTuningElevator),
                timingBudget = new LoggableTunedNumber("Elevator/TimingBudgetIndex", 2,
                        TuningConstants.isTuningElevator);

        public static com.ctre.phoenix6.CANBus CANBus =  Robot.everythingCanBus;
        public static RegionOfInterest regionOfInterest = new RegionOfInterest(1, 1, 1, 1);
        public static MotorVendor motorVendor = MotorVendor.CTRE_ON_CANIVORE;
        public static EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER;
        public static boolean leftInverted = true;
        public static boolean rightInverted = true;
        public static boolean isEncoderInverted = false;
        public static boolean isBrake = false;
        public static int kMotorLeftID = 40, kMotorRightID = 40, kLaserCANID = 27, currentLimit = 120;

        // must have position set in SysId
        public static DCMotor gearbox = DCMotor.getKrakenX60Foc(2);
        public static double elevatorGearing = 9,
                carriageMass = Units.lbsToKilograms(15.5),
                drumRadius = Units.inchesToMeters(1.5), // for SIM ONLY! (don't care about THAT accuracy)
                axleToMeters = (Units.inchesToMeters(.25) * 22 * 2) / (Math.PI * 2),
                maxPosition = Units.rotationsToRadians(10.5), // physically 59 butttt
                armLength = Units.inchesToMeters(5),
                physicalX = Units.inchesToMeters(6),
                physicalY = -Units.inchesToMeters(2.25),
                physicalZ = Units.inchesToMeters(2.813),
                elevatorMinHeight = Units.inchesToMeters(0),
                elevatorMaxHeight = 2, // physically 59 butttt
                distanceSensorFromValue = elevatorMinHeight - Units.inchesToMeters(2.675),
                distanceSensorStdDev = .003125;
        public static double elevatorSelfCheckMarginOfError = Units.inchesToMeters(4);
    }
}
