package frc.robot.utils.advancedMechs;

import com.ctre.phoenix6.CANBus;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Constants.EncoderType;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants.MotorVendor;

public class AdvancedMechanismConstants {
    public class PinkArm {
        public static CANBus CANBus = Robot.everythingCanBus;
        public static MotorVendor motorVendor = MotorVendor.CTRE_ON_CANIVORE;
        public static class ArmPosition {
            private double extensionLengthMeters = 0;
            private Rotation2d shoulderAngle = new Rotation2d();
            private Rotation2d wristAngle = new Rotation2d();

            public ArmPosition(double extensionLengthMeters, Rotation2d shoulderAngle, Rotation2d wristAngle) {
                this.extensionLengthMeters = extensionLengthMeters;
                this.shoulderAngle= shoulderAngle;
                this.wristAngle = wristAngle;
            }

            public Rotation2d getShoulderAngle() {
                return shoulderAngle;
            }

            public Rotation2d getWristAngle() {
                return wristAngle;
            }

            public double getExtensionLengthMeters() {
                return extensionLengthMeters;
            }
        }
        public static final ArmPosition zeroedArmPos = new ArmPosition(0.0, Rotation2d.fromDegrees(Shoulder.startingPosition), Rotation2d.fromDegrees(Wrist.startingPosition));
        public class Wrist{
            public static final int kMotorID = 25;
            public static final boolean inverted = false;
            //Will NEVER have an attached encoder.
            public static final double statorCurrentLimit = 80,
            supplyCurrentLimit = 30,
                    wristGearing = 1/((50.0 / 9.0) * (38.0 / 12.0) * (38.0 / 12.0)), //Be sure to double count the reverse chain.
                    wristMOI = 0.002, //not real val
                    startingPosition = Units.degreesToRadians(120),
                    maxPosition = Units.degreesToRadians(240),
                    wristLength = Units.inchesToMeters(6),
                    wristMass = Units.lbsToKilograms(6);
            public static final double wristPositionCoefficient = 2 * Math.PI / wristGearing;
            public static final LoggableTunedNumber kP = new LoggableTunedNumber("PinkArm/Wrist/kP", 5.0,
                    TuningConstants.isTuningPinkArm),
                    kI = new LoggableTunedNumber("PinkArm/Wrist/kI", 0.0, TuningConstants.isTuningPinkArm),
                    kD = new LoggableTunedNumber("PinkArm/Wrist/kD", 0, TuningConstants.isTuningPinkArm),
                    kS = new LoggableTunedNumber("PinkArm/Wrist/kS", 0, TuningConstants.isTuningPinkArm),
                    //No KG for wrist, as it is relative to gravity of shoulder position, and thats a lot more complex and not worth it
                    kV = new LoggableTunedNumber("PinkArm/Wrist/kV", 0, TuningConstants.isTuningPinkArm),
                    maxSpeed = new LoggableTunedNumber("PinkArm/Wrist/maxSpeedDegPerSec", 720, TuningConstants.isTuningPinkArm),
                    maxAcceleration = new LoggableTunedNumber("PinkArm/Wrist/maxAccelDegPerSec", 1440,
                            TuningConstants.isTuningPinkArm);
        }
        public class Shoulder {
            public static final boolean inverted = false;
            public static final boolean isEncoderInverted = false;
            public static final int kMotorFLID = 20, kMotorFRID = 21, kMotorBLID = 22, kMotorBRID = 23, kCANcoderID = 24;
            public static final EncoderType encoderType = EncoderType.NO_ATTACHED_ENCODER; // only other is CTRE
            public static final double statorCurrentLimit = 150,
            supplyCurrentLimit = 45,
                    shoulderGearing = 1/((60.0 / 12.0) * (50.0 / 36.0) * (50.0 / 18.0) * (58.0 / 10.0)),
                    encoderGearing = (58.0 / 10.0),
                    encoderOffsetRotations = .0,
                    shoulderMOI = 0.0925974241,
                    shoulderMOIDegreesFromZero = 0, //How many degrees, from the zero pos, is the MOI measured at? Used for gravity calc
                    //IMPORTANT TO NOTE, THAT "ZERO POS" IS HORIZONTAL ARM STRAIGHT OUT.
                    startingPosition = 0,
                    maxPosition = Units.degreesToRadians(135),
                    shoulderLength = Units.inchesToMeters(15),
                    shoulderMass = Units.lbsToKilograms(14);
            public static final double shoulderPositionCoefficient = 2 * Math.PI /shoulderGearing,
            shoulderEncoderPositionCoefficient = 2 * Math.PI / encoderGearing;
            public static final LoggableTunedNumber kP = new LoggableTunedNumber("PinkArm/Shoulder/kP", 5,
                    TuningConstants.isTuningPinkArm),
                    kI = new LoggableTunedNumber("PinkArm/Shoulder/kI", 0.000, TuningConstants.isTuningPinkArm),
                    kD = new LoggableTunedNumber("PinkArm/Shoulder/kD", 0.00, TuningConstants.isTuningPinkArm),
                    kS = new LoggableTunedNumber("PinkArm/Shoulder/kS", 0, TuningConstants.isTuningPinkArm),
                    kG = new LoggableTunedNumber("PinkArm/Shoulder/kG", 0, TuningConstants.isTuningPinkArm),
                    kV = new LoggableTunedNumber("PinkArm/Shoulder/kV", 0, TuningConstants.isTuningPinkArm),
                    maxSpeed = new LoggableTunedNumber("PinkArm/Shoulder/maxSpeedDegPerSec", 600, TuningConstants.isTuningPinkArm),
                    maxAcceleration = new LoggableTunedNumber("PinkArm/Shoulder/maxAccelDegPerSec", 1000,
                            TuningConstants.isTuningPinkArm);
        }
        public class Extension {
            public static final boolean inverted = false;
            public static final int kMotorOneID = 26, kMotorTwoID = 27;
            //Will NEVER have an attached encoder.
            public static final double 
            statorCurrentLimit = 120,
            supplyCurrentLimit = 60,
            extensionGearing = 1/((60.0 / 12.0) * (30.0 / 25.0)),
                    pulleyDiameter = Units.inchesToMeters(.25*16/Math.PI), //effective pulley diameter from sprocket
                    cascadeCoefficient = 2.0, //how much more extension you get from the second stage
                    extensionMOI = 0.002,
                    startingPosition = 0,
                    maxPosition = Units.inchesToMeters(45.282);
            public static final double extensionPositionCoefficient = Math.PI * pulleyDiameter / extensionGearing * cascadeCoefficient;
            public static final LoggableTunedNumber kP = new LoggableTunedNumber("PinkArm/Extension/kP", 5,
                    TuningConstants.isTuningPinkArm),
                    kI = new LoggableTunedNumber("PinkArm/Extension/kI", 0.0, TuningConstants.isTuningPinkArm),
                    kD = new LoggableTunedNumber("PinkArm/Extension/kD", 0.0, TuningConstants.isTuningPinkArm),
                    kS = new LoggableTunedNumber("PinkArm/Extension/kS", 0, TuningConstants.isTuningPinkArm),
                    kV = new LoggableTunedNumber("PinkArm/Extension/kV", 0.0, TuningConstants.isTuningPinkArm),
                    maxSpeed = new LoggableTunedNumber("PinkArm/Extension/maxSpeedMetersPerSec", 1.0,
                            TuningConstants.isTuningPinkArm),
                    maxAcceleration = new LoggableTunedNumber("PinkArm/Extension/maxAccelMetersPerSec", 2.0,
                            TuningConstants.isTuningPinkArm);
        }
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
