package frc.robot.subsystems.advancedMechs.PinkArm;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.advancedMechs.PinkArm.extension.ExtensionIO;
import frc.robot.subsystems.advancedMechs.PinkArm.extension.ExtensionIOInputsAutoLogged;
import frc.robot.subsystems.advancedMechs.PinkArm.shoulder.ShoulderIO;
import frc.robot.subsystems.advancedMechs.PinkArm.shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.subsystems.advancedMechs.PinkArm.wrist.WristIO;
import frc.robot.subsystems.advancedMechs.PinkArm.wrist.WristIOInputsAutoLogged;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.ArmPosition;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PinkArm extends SubsystemChecker {
    private ArmPosition wantedArmPose;
    private final LoggableTunedNumber shoulderSetpointToleranceDeg = new LoggableTunedNumber(
            "PinkArm/ShoulderSetpointToleranceDeg", 1, TuningConstants.isTuningPinkArm),
            extensionSetpointToleranceMeters = new LoggableTunedNumber("PinkArm/ExtensionSetpointToleranceMeters",
                    Units.inchesToMeters(0.5),
                    TuningConstants.isTuningPinkArm),
            wristSetpointToleranceDeg = new LoggableTunedNumber("PinkArm/WristSetpointToleranceDeg", 2,
                    TuningConstants.isTuningPinkArm),
            minToleranceForExtensionDegShoulder = new LoggableTunedNumber("PinkArm/MinToleranceForExtensionDegShoulder",
                    40, TuningConstants.isTuningPinkArm), // roughly be in the right position before extending
            extensionHomeMeters = new LoggableTunedNumber("PinkArm/ExtensionHomeMeters", 0, // slightly negative
                                                                                                // since we were PUSHING
                                                                                                // into the hard stop
                    TuningConstants.isTuningPinkArm),
            shoulderHomeDegrees = new LoggableTunedNumber("PinkArm/ShoulderHomeDegrees", 0,
                    TuningConstants.isTuningPinkArm),
            wristHomeDegrees = new LoggableTunedNumber("PinkArm/WristHomeDegrees", 135,
                    TuningConstants.isTuningPinkArm),
            extensionButtonHomeMeters = new LoggableTunedNumber("PinkArm/ExtensionButtonHomeMeters",
                    0, TuningConstants.isTuningPinkArm), // humans go to exact zero.
            shoulderButtonHomeDegrees = new LoggableTunedNumber("PinkArm/ShoulderButtonHomeDegrees", -.5, // slightly up
                                                                                                          // on
                                                                                                          // kickstand
                    TuningConstants.isTuningPinkArm),
            wristButtonHomeDegrees = new LoggableTunedNumber("PinkArm/WristButtonHomeDegrees", 135, // angled in
                    TuningConstants.isTuningPinkArm),
            extensionHomingDuty = new LoggableTunedNumber("PinkArm/ExtensionHomingDuty", -.07,
                    TuningConstants.isTuningPinkArm),
            shoulderHomingDuty = new LoggableTunedNumber("PinkArm/ShoulderHomingDuty", -.05,
                    TuningConstants.isTuningPinkArm),
            wristHomingDuty = new LoggableTunedNumber("PinkArm/WristHomingDuty", .07,
                    TuningConstants.isTuningPinkArm),
            zeroVelExtensionCutoff = new LoggableTunedNumber("PinkArm/ZeroVelExtensionCutoffMPS", 0.08,
                    TuningConstants.isTuningPinkArm),
            zeroVelShoulderCutoff = new LoggableTunedNumber("PinkArm/ZeroVelShoulderCutoffRPS", 0.08,
                    TuningConstants.isTuningPinkArm),

            zeroVelWristCutoff = new LoggableTunedNumber("PinkArm/ZeroVelWristCutoffRPS", 0.03,
                    TuningConstants.isTuningPinkArm),

            zeroVelTime = new LoggableTunedNumber("PinkArm/ZeroVelTime", 0.05, TuningConstants.isTuningPinkArm);
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

        wantedArmPose = AdvancedMechanismConstants.PinkArm.zeroedArmPos;
    }

    @Override
    public void periodic() {

        // update
        extensionIO.updateInputs(extensionInputs);
        shoulderIO.updateInputs(shoulderInputs);
        wristIO.updateInputs(wristInputs);
        Logger.processInputs("Arm/Extension", extensionInputs);
        Logger.processInputs("Arm/Shoulder", shoulderInputs);
        Logger.processInputs("Arm/Wrist", wristInputs);

        systemState = handleStateTransitions();

        Logger.recordOutput("Arm/SystemState", systemState);
        Logger.recordOutput("Arm/WantedState", wantedState);
        Logger.recordOutput("Arm/ReachedSetpoint", reachedSetpoint());

        double wantedExtensionMeters;
        Rotation2d wantedShoulderAngle;
        Rotation2d wantedWristAngle;

        if (wantedArmPose != null) {
            wantedShoulderAngle = wantedArmPose.getShoulderAngle();
            wantedExtensionMeters = wantedArmPose.getExtensionLengthMeters();
            wantedWristAngle = wantedArmPose.getWristAngle();
            Logger.recordOutput("Arm/WantedShoulderAngle", wantedShoulderAngle);
            Logger.recordOutput("Arm/WantedExtensionMeters", wantedExtensionMeters);
            Logger.recordOutput("Arm/WantedWristAngle", wantedWristAngle);
        }

        applyStates();

        previousWantedState = this.wantedState;

    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (previousWantedState != WantedState.HOME) {
                    isExtensionAndShoulderHomed = false;
                    isWristHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(extensionInputs.extensionVelocityMetersPerSec) <= zeroVelExtensionCutoff.get()
                            && Math.abs(shoulderInputs.shoulderAngularVelocityRadPerSec) <= zeroVelShoulderCutoff
                                    .get()) {
                        if (Double.isNaN(extensionAndShoulderHomeTimeStamp)) {
                            extensionAndShoulderHomeTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING_SHOULDER_AND_EXTENSION;
                        } else if ((Timer.getFPGATimestamp() - extensionAndShoulderHomeTimeStamp) >= zeroVelTime
                                .get()) {

                            if (!hasInitialHomeCompleted) {
                                hasInitialHomeCompleted = true;
                                tareExtensionAndShoulder();
                                return SystemState.HOMING_WRIST;
                            }

                            if (Math.abs(wristInputs.wristAngularVelocityRadPerSec) <= zeroVelWristCutoff.get()) {
                                if (Double.isNaN(wristHomeTimeStamp)) {
                                    wristHomeTimeStamp = Timer.getFPGATimestamp();
                                    return SystemState.HOMING_WRIST;
                                } else if ((Timer.getFPGATimestamp() - wristHomeTimeStamp) >= zeroVelTime.get()) {

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
                extensionIO.setDutyCycle(extensionHomingDuty.get());
                shoulderIO.setDutyCycle(shoulderHomingDuty.get());
                break;
            case HOMING_WRIST:
                wristIO.setDutyCycle(wristHomingDuty.get());
                extensionIO.setDutyCycle(extensionHomingDuty.get());
                shoulderIO.setDutyCycle(shoulderHomingDuty.get());
                break;
            case IDLING:
                extensionIO.setDutyCycle(0);
                shoulderIO.setDutyCycle(0);
                wristIO.setDutyCycle(0);
                break;
            case MOVING_TO_POSITION:
                if (isExtensionAndShoulderHomed && isWristHomed) {

                    shoulderIO.setTargetAngle(wantedArmPose.getShoulderAngle());
                    if (!MathUtil.isNear(
                            wantedArmPose.getShoulderAngle().getDegrees(),
                            shoulderInputs.shoulderAngle.getDegrees(),
                            minToleranceForExtensionDegShoulder.get())) { // if we are not close enough to where the
                        // shoulder is supposed to be, keep the
                        // extension at zero in order to avoid collisions
                        extensionIO.setTargetExtension(0);
                    } else {
                        extensionIO.setTargetExtension(wantedArmPose.getExtensionLengthMeters());
                    }

                    if (Units.metersToInches(extensionInputs.extensionPositionInMeters) <= 27.0) {// 180 = flat, 130ish
                                                                                                  // = facing IN bot.
                        wristIO.setTargetAngle(Rotation2d.fromDegrees(
                                Math.min(wantedArmPose.getWristAngle().getDegrees(), 130.0)));
                    } else {
                        wristIO.setTargetAngle(wantedArmPose.getWristAngle());
                    }
                }
                break;
        }
    }

    public void tareAllAxesUsingButtonValues() {

        isExtensionAndShoulderHomed = true;
        isWristHomed = true;
        extensionIO.resetExtensionPosition(extensionButtonHomeMeters.get());
        shoulderIO.resetShoulderAngle(
                Rotation2d.fromDegrees(shoulderButtonHomeDegrees.get()));
        wristIO.resetWristAngle(Rotation2d.fromDegrees(wristButtonHomeDegrees.get()));

    }

    public void tareAllAxes() {
        tareWrist();
        tareExtensionAndShoulder();
    }

    public void tareWrist() {
        isWristHomed = true;
        wristIO.resetWristAngle(Rotation2d.fromDegrees(wristHomeDegrees.get()));

    }

    public void tareExtensionAndShoulder() {
        isExtensionAndShoulderHomed = true;
        extensionIO.resetExtensionPosition(extensionHomeMeters.get());
        shoulderIO.resetShoulderAngle(Rotation2d.fromDegrees(shoulderHomeDegrees.get()));

    }

    public void setBrakeMode(boolean brakeModeEnabled) {

        extensionIO.setBrakeMode(brakeModeEnabled);
        shoulderIO.setBrakeMode(brakeModeEnabled);
        wristIO.setBrakeMode(brakeModeEnabled);

    }

    public boolean hasHomeCompleted() {
        return isExtensionAndShoulderHomed && isWristHomed;
    }

    public double getCurrentExtensionPositionInMeters() {
        return extensionInputs.extensionPositionInMeters;

    }

    public Rotation2d getCurrentShoulderPosition() {
        return shoulderInputs.shoulderAngle;

    }

    public Rotation2d getCurrentWristPosition() {
        return wristInputs.wristAngle;

    }

    public ArmPosition getWantedArmPose() {
        return wantedArmPose;
    }

    public boolean reachedSetpoint() {

        return MathUtil.isNear(
                wantedArmPose.getShoulderAngle().getDegrees(),
                shoulderInputs.shoulderAngle.getDegrees(),
                shoulderSetpointToleranceDeg.get())
                && MathUtil.isNear(
                        wantedArmPose.getExtensionLengthMeters(),
                        extensionInputs.extensionPositionInMeters,
                        extensionSetpointToleranceMeters.get())
                && MathUtil.isNear(
                        wantedArmPose.getWristAngle().getDegrees(),
                        wristInputs.wristAngle.getDegrees(),
                        wristSetpointToleranceDeg.get());

    }

    public boolean reachedSetpoint(ArmPosition armPosition) {

        return MathUtil.isNear(
                armPosition.getShoulderAngle().getDegrees(),
                shoulderInputs.shoulderAngle.getDegrees(),
                shoulderSetpointToleranceDeg.get())
                && MathUtil.isNear(
                        armPosition.getExtensionLengthMeters(),
                        extensionInputs.extensionPositionInMeters,
                        extensionSetpointToleranceMeters.get())
                && MathUtil.isNear(
                        armPosition.getWristAngle().getDegrees(),
                        wristInputs.wristAngle.getDegrees(),
                        wristSetpointToleranceDeg.get());

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
        Rotation2d wantedWrist = wantedArmPose.getWristAngle();
        this.wantedArmPose = new ArmPosition(
                armPosition.getExtensionLengthMeters(), armPosition.getShoulderAngle(), wantedWrist);
    }

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> extensionHardware = extensionIO.getSelfCheckingHardware();
        for (SelfChecking motor : extensionHardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        List<SelfChecking> shoulderHardware = shoulderIO.getSelfCheckingHardware();
        for (SelfChecking motor : shoulderHardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        List<SelfChecking> wristHardware = wristIO.getSelfCheckingHardware();
        for (SelfChecking motor : wristHardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        double totalCurrent = 0.0;
        totalCurrent += Math.abs(extensionInputs.extensionStatorCurrentAmps);
        totalCurrent += Math.abs(shoulderInputs.shoulderStatorCurrentAmps);
        totalCurrent += Math.abs(wristInputs.wristStatorCurrentAmps);
        return totalCurrent;
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> tempMap = new HashMap<>();
        tempMap.put("ExtensionOneMotorTemp", extensionInputs.extensionOneMotorTemp);
        tempMap.put("ExtensionTwoMotorTemp", extensionInputs.extensionTwoMotorTemp);
        tempMap.put("ShoulderFRMotorTemp", shoulderInputs.shoulderFRMotorTemp);
        tempMap.put("ShoulderFLMotorTemp", shoulderInputs.shoulderFLMotorTemp);
        tempMap.put("ShoulderBRMotorTemp", shoulderInputs.shoulderBRMotorTemp);
        tempMap.put("ShoulderBLMotorTemp", shoulderInputs.shoulderBLMotorTemp);
        tempMap.put("WristMotorTemp", wristInputs.wristMotorTemp);
        return tempMap;
    }

    public void setAllCurrentLimit(int extensionAmps, int shoulderAmps, int wristAmps) {
        extensionIO.setCurrentLimit(extensionAmps);
        shoulderIO.setCurrentLimit(shoulderAmps);
        wristIO.setCurrentLimit(wristAmps);
    }

    @Override
    public void setCurrentLimit(int extensionAmps) {
        setAllCurrentLimit(extensionAmps, 0, 0);
    }

    @Override
    protected Command systemCheckCommand() {
        return Commands.none(); // This system is too mechanically complex for a simple system check command.
                                // Return a no-op.
    }
}