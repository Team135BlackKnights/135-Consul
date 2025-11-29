package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import static frc.robot.utils.SparkUtil.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSparkBase implements ModuleIO {
        private final Rotation2d zeroRotation;

        // Hardware objects
        private final SparkBase driveSpark;
        private final SparkBase turnSpark;
        private final RelativeEncoder driveEncoder;
        private final SparkAnalogSensor turnEncoder;
        private final boolean turnInverted;
        private final boolean turnEncoderInverted;
        private final boolean driveInverted;
        // Closed loop controllers
        private final SparkClosedLoopController driveController;
        private final SparkClosedLoopController turnController;

        // Queue inputs from odometry thread
        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;

        // Connection debouncers
        private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
        private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
        private String moduleName = "UNFINISHED";
        SparkBaseConfig driveConfig;
        SparkBaseConfig turnConfig;

        public ModuleIOSparkBase(int module) {
                switch (module) {
                        case 0:
                                moduleName = "Front Left";
                                zeroRotation = new Rotation2d(DriveConstants.kFrontLeftAbsEncoderOffsetRad);
                                driveSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                turnSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kFrontLeftTurningPort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kFrontLeftTurningPort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                driveInverted = DriveConstants.kFrontLeftDriveReversed;
                                turnInverted = DriveConstants.kFrontLeftTurningReversed;
                                turnEncoderInverted = DriveConstants.kFrontLeftAbsEncoderReversed;
                                break;
                        case 1:
                                moduleName = "Front Right";
                                zeroRotation = new Rotation2d(DriveConstants.kFrontRightAbsEncoderOffsetRad);
                                driveSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                turnSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kFrontRightTurningPort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kFrontRightTurningPort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                driveInverted = DriveConstants.kFrontRightDriveReversed;
                                turnInverted = DriveConstants.kFrontRightTurningReversed;
                                turnEncoderInverted = DriveConstants.kFrontRightAbsEncoderReversed;
                                break;
                        case 2:
                                moduleName = "Back Left";
                                zeroRotation = new Rotation2d(DriveConstants.kBackLeftAbsEncoderOffsetRad);
                                driveSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                turnSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kBackLeftTurningPort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kBackLeftTurningPort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                driveInverted = DriveConstants.kBackLeftDriveReversed;

                                turnInverted = DriveConstants.kBackLeftTurningReversed;
                                turnEncoderInverted = DriveConstants.kBackLeftAbsEncoderReversed;

                                break;
                        case 3:
                                moduleName = "Back Right";
                                zeroRotation = new Rotation2d(DriveConstants.kBackRightAbsEncoderOffsetRad);
                                driveSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                turnSpark = switch (DriveConstants.robotMotorController) {
                                        case NEO_SPARK_MAX -> new SparkFlex(
                                                        DriveConstants.kBackRightTurningPort, MotorType.kBrushless);
                                        case VORTEX_SPARK_FLEX -> new SparkMax(
                                                        DriveConstants.kBackRightTurningPort, MotorType.kBrushless);
                                        default -> throw new IllegalArgumentException("Invalid motor controller type");
                                };
                                driveInverted = DriveConstants.kBackRightDriveReversed;
                                turnInverted = DriveConstants.kBackRightTurningReversed;
                                turnEncoderInverted = DriveConstants.kBackRightAbsEncoderReversed;
                                break;
                        default:

                                throw new IllegalArgumentException("Invalid module number: " + module);
                }
                driveEncoder = driveSpark.getEncoder();
                turnEncoder = turnSpark.getAnalog();
                driveController = driveSpark.getClosedLoopController();
                turnController = turnSpark.getClosedLoopController();
                // Configure drive motor
                driveConfig = new SparkFlexConfig();
                driveConfig
                                .inverted(driveInverted)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(DriveConstants.kMaxDriveCurrent)
                                .voltageCompensation(12.0);
                driveConfig.encoder
                                .positionConversionFactor(
                                                2 * Math.PI / DriveConstants.TrainConstants.kDriveMotorGearRatioLow)
                                .velocityConversionFactor((2 * Math.PI) / 60.0
                                                / DriveConstants.TrainConstants.kDriveMotorGearRatioLow)
                                .uvwMeasurementPeriod(10)
                                .uvwAverageDepth(2);
                driveConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(DriveConstants.overallTurningMotorConstantContainer.getP(),
                                                DriveConstants.overallTurningMotorConstantContainer.getI(),
                                                DriveConstants.overallTurningMotorConstantContainer.getD());
                driveConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.TrainConstants.odomHz))
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                tryUntilOk(
                                driveSpark,
                                5,
                                () -> driveSpark.configure(
                                                driveConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));
                tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

                // Configure turn motor
                turnConfig = new SparkMaxConfig();
                turnConfig
                                .inverted(turnInverted)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(DriveConstants.kMaxTurnCurrent)
                                .voltageCompensation(12.0);
                turnConfig.analogSensor
                                .inverted(turnEncoderInverted)
                                .positionConversionFactor(2 * Math.PI)
                                .velocityConversionFactor(2 * Math.PI / 60);
                turnConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAnalogSensor)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(-Math.PI * RobotController.getVoltage3V3(),
                                                Math.PI * RobotController.getVoltage3V3())
                                .pid(DriveConstants.overallTurningMotorConstantContainer.getP(),
                                                DriveConstants.overallTurningMotorConstantContainer.getI(),
                                                DriveConstants.overallTurningMotorConstantContainer.getD());
                turnConfig.signals
                                .analogPositionAlwaysOn(true)
                                .analogPositionPeriodMs((int) (1000.0 / DriveConstants.TrainConstants.odomHz))
                                .analogVelocityAlwaysOn(true)
                                .analogVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                tryUntilOk(
                                turnSpark,
                                5,
                                () -> turnSpark.configure(
                                                turnConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kPersistParameters));

                // Create odometry queues
                drivePositionQueue = OdometryThread.registerInput(driveEncoder::getPosition);
                turnPositionQueue = OdometryThread.registerInput(turnEncoder::getPosition);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {
                // Update drive inputs
                sparkStickyFault = false;
                ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRads = value);
                ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadsPerSec = value);
                ifOk(
                                driveSpark,
                                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
                ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveSupplyCurrentAmps = value);
                inputs.driveMotorConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

                // Update turn inputs
                sparkStickyFault = false;
                ifOk(
                                turnSpark,
                                turnEncoder::getPosition,
                                (value) -> {
                                        inputs.turnPosition = new Rotation2d(value).div(RobotController.getVoltage3V3())
                                                        .minus(zeroRotation);
                                        inputs.turnAbsolutePosition = new Rotation2d(value)
                                                        .div(RobotController.getVoltage3V3());
                                });
                ifOk(turnSpark, turnEncoder::getVelocity,
                                (value) -> inputs.turnVelocityRadsPerSec = value / RobotController.getVoltage3V3());
                ifOk(
                                turnSpark,
                                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
                ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnSupplyCurrentAmps = value);
                inputs.turnMotorConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

                // Update odometry inputs
                inputs.odometryDrivePositionsMeters = drivePositionQueue.stream().mapToDouble(
                                (Double value) -> value * DriveConstants.TrainConstants.kWheelDiameter.get() / 2)
                                .toArray();
                inputs.odometryTurnPositions = turnPositionQueue.stream()
                                .map((Double value) -> new Rotation2d(value).div(RobotController.getVoltage3V3())
                                                .minus(zeroRotation))
                                .toArray(Rotation2d[]::new);
                drivePositionQueue.clear();
                turnPositionQueue.clear();
        }

        @Override
        public void runDriveVolts(double output) {
                driveSpark.setVoltage(output);
        }

        @Override
        public void runTurnVolts(double output) {
                turnSpark.setVoltage(output);
        }

        @Override
        public void runDriveVelocitySetpoint(double velocityRadPerSec, double feedForward) {
                driveController.setSetpoint(
                                velocityRadPerSec,
                                ControlType.kVelocity,
                                ClosedLoopSlot.kSlot0,
                                feedForward,
                                ArbFFUnits.kVoltage);
        }

        @Override
        public void runTurnPositionSetpoint(double rotation) {
                /*
                 * double volts = turnPidController
                 * .calculate(MathUtil.inputModulus(
                 * turnEncoder.getPosition() / RobotController.getVoltage3V3()
                 * - zeroRotation.getRadians(),
                 * -Math.PI, Math.PI), MathUtil.inputModulus(rotation, -Math.PI,Math.PI))
                 * + turnFF.calculate(turnEncoder.getVelocity() /
                 * RobotController.getVoltage3V3());
                 */
                double pos = MathUtil.inputModulus(
                                (rotation + zeroRotation.getRadians()) * RobotController.getVoltage3V3(),
                                -Math.PI * RobotController.getVoltage3V3(), Math.PI * RobotController.getVoltage3V3());
                turnController.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        /** Configure drive PID */
        @Override
        public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
                driveConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(
                                                kP,
                                                kI,
                                                kD);
                driveConfig.closedLoop.feedForward.kV(kV);
                driveSpark.configureAsync(
                                                driveConfig, ResetMode.kResetSafeParameters,
                                                PersistMode.kNoPersistParameters);
        }

        /** Configure turn PID */
        @Override
        public void setTurnPID(double kP, double kI, double kD, double kS, double kV, double deadbandAmps) {
                turnConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAnalogSensor)
                                .positionWrappingEnabled(true)
                                .positionWrappingInputRange(-Math.PI * RobotController.getVoltage3V3(),
                                                Math.PI * RobotController.getVoltage3V3())
                                .pid(
                                                kP,
                                                kI, kD);
                turnConfig.closedLoop.feedForward.kS(kS);
                turnSpark.configureAsync(
                                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        /** Enable or disable brake mode on the drive motor. */
        @Override
        public void setDriveBrakeMode(boolean enable) {
                if (enable) {
                        driveConfig.idleMode(IdleMode.kBrake);
                } else {
                        driveConfig.idleMode(IdleMode.kCoast);
                }
                driveSpark.configureAsync(
                                driveConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
        }

        /** Enable or disable brake mode on the turn motor. */
        @Override
        public void setTurnBrakeMode(boolean enable) {
                if (enable) {
                        turnConfig.idleMode(IdleMode.kBrake);
                } else {
                        turnConfig.idleMode(IdleMode.kCoast);
                }
                turnSpark.configureAsync(
                                turnConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
        }

        /** Update the motor controllers to a specificed max amperage */
        @Override
        public void setCurrentLimit(int amps) {
                driveConfig.smartCurrentLimit(amps);
                driveSpark.configureAsync(
                                driveConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

        }

        /** Disable output to all motors */
        @Override
        public void stop() {
                runDriveVolts(0.0);
                runTurnVolts(0.0);
        }

        @Override
        public List<SelfChecking> getSelfCheckingHardware() {
                List<SelfChecking> hardware = new ArrayList<SelfChecking>();
                hardware.add(new SelfCheckingSparkBase(moduleName + "Drive", driveSpark));
                hardware.add(new SelfCheckingSparkBase(moduleName + "Turn", turnSpark));
                return hardware;
        }
}
