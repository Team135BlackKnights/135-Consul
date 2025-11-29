package frc.robot.subsystems.advancedMechs.PinkArm;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemChecker;

import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PinkArm{

    /*private ArmPosition wantedArmPose;

    private final ExtensionIO extensionIO;
    private final ShoulderIO shoulderIO;
    private final WristIO wristIO;

    private final ExtensionIOInputsAutoLogged extensionInputs = new ExtensionIOInputsAutoLogged();
    private final ShoulderIOInputsAutoLogged shoulderInputs = new ShoulderIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private double extensionAndShoulderHomeTimeStamp = Double.NaN;
    private double wristHomeTimeStamp = Double.NaN;
    private boolean isExtensionAndShoulderHomed = false;
    private boolean isWristHomed = false;

    private boolean hasInitialHomeCompleted = false;

    public enum WantedState {
        HOME,
        IDLE,
        MOVE_TO_POSITION,
    }

    private enum SystemState {
        HOMING_SHOULDER_AND_EXTENSION,
        HOMING_WRIST,
        IDLING,
        MOVING_TO_POSITION
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public PinkArm(ExtensionIO extensionIO, ShoulderIO shoulderIO, WristIO wristIO) {
        this.extensionIO = extensionIO;
        this.shoulderIO = shoulderIO;
        this.wristIO = wristIO;

        extensionIO.setShoulderAngleSupplier(() -> shoulderInputs.shoulderAngle);
        wantedArmPose = ArmPoseConstants.ZEROED;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {
                    synchronized (extensionInputs) {
                        synchronized (shoulderInputs) {
                            synchronized (wristInputs) {
                                extensionIO.updateInputs(extensionInputs);
                                shoulderIO.updateInputs(shoulderInputs);
                                wristIO.updateInputs(wristInputs);
                            }
                        }
                    }
                },
                extensionIO,
                shoulderIO,
                wristIO);
    }

    @Override
    public void periodic() {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                synchronized (wristInputs) {
                    Logger.processInputs("Subsystems/Arm/Extension", extensionInputs);
                    Logger.processInputs("Subsystems/Arm/Shoulder", shoulderInputs);
                    Logger.processInputs("Subsystems/Arm/Wrist", wristInputs);

                    systemState = handleStateTransitions();

                    Logger.recordOutput("Subsystems/Arm/SystemState", systemState);
                    Logger.recordOutput("Subsystems/Arm/WantedState", wantedState);
                    Logger.recordOutput("Subsystems/Arm/ReachedSetpoint", reachedSetpoint());

                    double wantedExtensionMeters;
                    Rotation2d wantedShoulderAngle;
                    Rotation2d wantedWristAngle;

                    if (wantedArmPose != null) {
                        wantedShoulderAngle = wantedArmPose.getShoulderAngleRot2d();
                        wantedExtensionMeters = wantedArmPose.getExtensionLengthMeters();
                        wantedWristAngle = wantedArmPose.getWristAngleRot2d();
                        Logger.recordOutput("Subsystems/Arm/WantedShoulderAngle", wantedShoulderAngle);
                        Logger.recordOutput("Subsystems/Arm/WantedExtensionMeters", wantedExtensionMeters);
                        Logger.recordOutput("Subsystems/Arm/WantedWristAngle", wantedWristAngle);
                    }

                    applyStates();

                    previousWantedState = this.wantedState;
                }
            }
        }
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (previousWantedState != WantedState.HOME) {
                    isExtensionAndShoulderHomed = false;
                    isWristHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(extensionInputs.extensionVelocityMetersPerSec)
                                    <= ArmConstants.EXTENSION_ZERO_VELOCITY_THRESHOLD_METERS_PER_SECOND
                            && Math.abs(shoulderInputs.shoulderAngularVelocityRadPerSec)
                                    <= ArmConstants.SHOULDER_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                        if (Double.isNaN(extensionAndShoulderHomeTimeStamp)) {
                            extensionAndShoulderHomeTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING_SHOULDER_AND_EXTENSION;
                        } else if ((Timer.getFPGATimestamp() - extensionAndShoulderHomeTimeStamp)
                                >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                            if (!hasInitialHomeCompleted) {
                                hasInitialHomeCompleted = true;
                                tareExtensionAndShoulder();
                                return SystemState.HOMING_WRIST;
                            }

                            if (Math.abs(wristInputs.wristAngularVelocityRadPerSec)
                                    <= ArmConstants.WRIST_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                                if (Double.isNaN(wristHomeTimeStamp)) {
                                    wristHomeTimeStamp = Timer.getFPGATimestamp();
                                    return SystemState.HOMING_WRIST;
                                } else if ((Timer.getFPGATimestamp() - wristHomeTimeStamp)
                                        >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                                    isWristHomed = true;
                                    wristHomeTimeStamp = Double.NaN;
                                    isExtensionAndShoulderHomed = true;
                                    extensionAndShoulderHomeTimeStamp = Double.NaN;

                                    tareWrist();
                                    hasInitialHomeCompleted = false;

                                    setWantedState(WantedState.IDLE);
                                    return SystemState.IDLING;

                                } else {
                                    return SystemState.HOMING_WRIST;
                                }
                            } else {
                                wristHomeTimeStamp = Double.NaN;
                                return SystemState.HOMING_WRIST;
                            }
                        } else {
                            return SystemState.HOMING_SHOULDER_AND_EXTENSION;
                        }
                    } else {
                        extensionAndShoulderHomeTimeStamp = Double.NaN;
                        return SystemState.HOMING_SHOULDER_AND_EXTENSION;
                    }
                } else {
                    return SystemState.HOMING_SHOULDER_AND_EXTENSION;
                }
            case IDLE:
                return SystemState.IDLING;
            case MOVE_TO_POSITION:
                return SystemState.MOVING_TO_POSITION;
        }
        return SystemState.IDLING;
    }

    public void applyStates() {
        switch (systemState) {
            case HOMING_SHOULDER_AND_EXTENSION:
                extensionIO.setDutyCycle(ArmConstants.EXTENSION_ZEROING_DUTY_CYCLE);
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case HOMING_WRIST:
                wristIO.setDutyCycle(ArmConstants.WRIST_ZEROING_DUTY_CYCLE);
                extensionIO.setDutyCycle(ArmConstants.EXTENSION_ZEROING_DUTY_CYCLE);
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case IDLING:
                extensionIO.setDutyCycle(0);
                shoulderIO.setDutyCycle(0);
                wristIO.setDutyCycle(0);
                break;
            case MOVING_TO_POSITION:
                if (isExtensionAndShoulderHomed && isWristHomed) {
                    if ((extensionInputs.extensionPositionInMeters - wantedArmPose.getExtensionLengthMeters())
                                    > Units.inchesToMeters(5.0)
                            && extensionInputs.extensionPositionInMeters
                                    >= Units.inchesToMeters(35.0)) { // if coming down from L4

                        if (shoulderInputs.shoulderAngle.getDegrees() >= 93.0) { // if scoring over back
                            shoulderIO.setTargetAngle(Rotation2d.fromDegrees(91.0));
                        } else if (shoulderInputs.shoulderAngle.getDegrees() <= 73.0) { // if scoring over front
                            shoulderIO.setTargetAngle(Rotation2d.fromDegrees(75.0));
                        } else { // do this after the shoulder has been moved within the robot
                            extensionIO.setTargetExtension(wantedArmPose.getExtensionLengthMeters());
                        }
                    } else {
                        shoulderIO.setTargetAngle(wantedArmPose.getShoulderAngleRot2d());
                        if (!MathUtil.isNear(
                                wantedArmPose.getShoulderAngleRot2d().getDegrees(),
                                shoulderInputs.shoulderAngle.getDegrees(),
                                ArmConstants
                                        .TOLERANCE_FOR_EXTENSION_DEGREES)) { // if we are not close enough to where the
                            // shoulder is supposed to be, keep the
                            // extension at zero
                            extensionIO.setTargetExtension(0);
                        } else {
                            extensionIO.setTargetExtension(wantedArmPose.getExtensionLengthMeters());
                        }
                    }

                    if (Units.metersToInches(extensionInputs.extensionPositionInMeters) <= 27.0) {
                        wristIO.setTargetAngle(Rotation2d.fromDegrees(
                                Math.min(wantedArmPose.getWristAngleRot2d().getDegrees(), 130.0)));
                    } else {
                        wristIO.setTargetAngle(wantedArmPose.getWristAngleRot2d());
                    }
                }
                break;
        }
    }

    public void tareAllAxesUsingButtonValues() {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                synchronized (wristInputs) {
                    isExtensionAndShoulderHomed = true;
                    isWristHomed = true;
                    extensionIO.resetExtensionPosition(ArmConstants.EXTENSION_BUTTON_HOME_LENGTH_METERS);
                    shoulderIO.resetShoulderAngle(
                            Rotation2d.fromDegrees(ArmConstants.SHOULDER_BUTTON_HOME_ANGLE_DEGREES));
                    wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_BUTTON_HOME_ANGLE_DEGREES));
                }
            }
        }
    }

    public void tareAllAxes() {
        tareWrist();
        tareExtensionAndShoulder();
    }

    public void tareWrist() {
        synchronized (wristInputs) {
            isWristHomed = true;
            wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_DRIVEN_HOME_ANGLE_DEGREES));
        }
    }

    public void tareExtensionAndShoulder() {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                isExtensionAndShoulderHomed = true;
                extensionIO.resetExtensionPosition(ArmConstants.EXTENSION_DRIVEN_HOME_LENGTH_METERS);
                shoulderIO.resetShoulderAngle(Rotation2d.fromDegrees(ArmConstants.SHOULDER_DRIVEN_HOME_ANGLE_DEGREES));
            }
        }
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                synchronized (wristInputs) {
                    extensionIO.setNeutralMode(neutralModeValue);
                    shoulderIO.setNeutralMode(neutralModeValue);
                    wristIO.setNeutralMode(neutralModeValue);
                }
            }
        }
    }

    public boolean hasHomeCompleted() {
        return isExtensionAndShoulderHomed && isWristHomed;
    }

    public double getCurrentExtensionPositionInMeters() {
        synchronized (extensionInputs) {
            return extensionInputs.extensionPositionInMeters;
        }
    }

    public Rotation2d getCurrentShoulderPosition() {
        synchronized (shoulderInputs) {
            return shoulderInputs.shoulderAngle;
        }
    }

    public Rotation2d getCurrentWristPosition() {
        synchronized (wristInputs) {
            return wristInputs.wristAngle;
        }
    }

    public ArmPosition getWantedArmPose() {
        return wantedArmPose;
    }

    public boolean reachedSetpoint() {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                synchronized (wristInputs) {
                    return MathUtil.isNear(
                                    wantedArmPose.getShoulderAngleRot2d().getDegrees(),
                                    shoulderInputs.shoulderAngle.getDegrees(),
                                    ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                            && MathUtil.isNear(
                                    wantedArmPose.getExtensionLengthMeters(),
                                    extensionInputs.extensionPositionInMeters,
                                    ArmConstants.EXTENSION_SETPOINT_TOLERANCE_METERS)
                            && MathUtil.isNear(
                                    wantedArmPose.getWristAngleRot2d().getDegrees(),
                                    wristInputs.wristAngle.getDegrees(),
                                    ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
                }
            }
        }
    }

    public boolean reachedSetpoint(ArmPosition armPosition) {
        synchronized (extensionInputs) {
            synchronized (shoulderInputs) {
                synchronized (wristInputs) {
                    return MathUtil.isNear(
                                    armPosition.getShoulderAngleRot2d().getDegrees(),
                                    shoulderInputs.shoulderAngle.getDegrees(),
                                    ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                            && MathUtil.isNear(
                                    armPosition.getExtensionLengthMeters(),
                                    extensionInputs.extensionPositionInMeters,
                                    ArmConstants.EXTENSION_SETPOINT_TOLERANCE_METERS)
                            && MathUtil.isNear(
                                    armPosition.getWristAngleRot2d().getDegrees(),
                                    wristInputs.wristAngle.getDegrees(),
                                    ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
                }
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        this.wantedArmPose = armPosition;
    }

    public void setOnlyExtensionAndShoulder(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        var wantedWrist = wantedArmPose.getWristAngleRot2d();
        this.wantedArmPose = new ArmPosition(
                armPosition.getExtensionLengthMeters(), armPosition.getShoulderAngleRot2d(), wantedWrist);
    }

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOrchestraDevices'");
    }

    @Override
    public double getCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrent'");
    }

    @Override
    public HashMap<String, Double> getTemps() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTemps'");
    }

    @Override
    public void setCurrentLimit(int amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
    }

    @Override
    protected Command systemCheckCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'systemCheckCommand'");
    }*/
}