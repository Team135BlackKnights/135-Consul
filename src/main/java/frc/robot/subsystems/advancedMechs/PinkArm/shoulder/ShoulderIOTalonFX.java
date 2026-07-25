package frc.robot.subsystems.advancedMechs.PinkArm.shoulder;


import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.EncoderType;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Shoulder;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class ShoulderIOTalonFX implements ShoulderIO {
    public final TalonFX shoulderFR;
    public final TalonFX shoulderFL;
    public final TalonFX shoulderBR;
    public final TalonFX shoulderBL;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderSupplyCurrentAmps;
    private final StatusSignal<Current> shoulderStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> shoulderAngularVelocityRadPerSec;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccelerationRadPerSecSquared;
    private final StatusSignal<Temperature> shoulderMotorFRTemp;
    private final StatusSignal<Temperature> shoulderMotorFLTemp;
    private final StatusSignal<Temperature> shoulderMotorBRTemp;
    private final StatusSignal<Temperature> shoulderMotorBLTemp;
    private final TalonFXConfiguration config;
    private final boolean usingCanCoder = Shoulder.encoderType == EncoderType.CTRE;
    private double positionCoefficient;

    public ShoulderIOTalonFX() {
        shoulderFR = new TalonFX(Shoulder.kMotorFRID,
                AdvancedMechanismConstants.PinkArm.CANBus);
        shoulderFL = new TalonFX(Shoulder.kMotorFLID,
                AdvancedMechanismConstants.PinkArm.CANBus);
        shoulderBR = new TalonFX(Shoulder.kMotorBRID,
                AdvancedMechanismConstants.PinkArm.CANBus);
        shoulderBL = new TalonFX(Shoulder.kMotorBLID,
                AdvancedMechanismConstants.PinkArm.CANBus);

        Follower followerWithoutInverse = new Follower(shoulderFR.getDeviceID(), MotorAlignmentValue.Aligned);
        Follower followerWithInverse = new Follower(shoulderFR.getDeviceID(), MotorAlignmentValue.Opposed);

        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Shoulder.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Shoulder.statorCurrentLimit;

        config.Slot0.kP = Shoulder.kP.get();
        config.Slot0.kI = Shoulder.kI.get();
        config.Slot0.kD = Shoulder.kD.get();

        config.Slot0.kS = Shoulder.kS.get();
        config.Slot0.kG = Shoulder.kG.get();
        config.Slot0.kV = Shoulder.kV.get();
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.GravityArmPositionOffset = Units.degreesToRotations(Shoulder.shoulderMOIDegreesFromZero);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Encoder setup, if applicable
        positionCoefficient = Shoulder.shoulderPositionCoefficient;
        if (usingCanCoder) {
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
            config.Feedback.FeedbackRemoteSensorID = AdvancedMechanismConstants.PinkArm.Shoulder.kCANcoderID;
            positionCoefficient = Shoulder.shoulderEncoderPositionCoefficient;
        }
        config.MotionMagic.MotionMagicAcceleration = Shoulder.maxAcceleration.get()
                / positionCoefficient;
        config.MotionMagic.MotionMagicCruiseVelocity = Shoulder.maxSpeed.get() / positionCoefficient;
        config.MotorOutput.Inverted = Shoulder.inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        shoulderFR.getConfigurator().apply(config);
        shoulderFL.getConfigurator().apply(config);
        shoulderBR.getConfigurator().apply(config);
        shoulderBL.getConfigurator().apply(config);

        shoulderFL.setControl(followerWithInverse);
        shoulderBL.setControl(followerWithInverse);
        shoulderBR.setControl(followerWithoutInverse);

        shoulderAngle = shoulderFR.getPosition();
        shoulderAppliedVolts = shoulderFR.getMotorVoltage();
        shoulderSupplyCurrentAmps = shoulderFR.getSupplyCurrent();
        shoulderStatorCurrentAmps = shoulderFR.getStatorCurrent();
        shoulderAngularVelocityRadPerSec = shoulderFR.getRotorVelocity();
        shoulderAngularAccelerationRadPerSecSquared = shoulderFR.getAcceleration();
        shoulderMotorFRTemp = shoulderFR.getDeviceTemp();
        shoulderMotorFLTemp = shoulderFL.getDeviceTemp();
        shoulderMotorBRTemp = shoulderBR.getDeviceTemp();
        shoulderMotorBLTemp = shoulderBL.getDeviceTemp();

    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            double magicAccel = Shoulder.maxAcceleration.get()
                    / positionCoefficient;
            double magicCruise = Shoulder.maxSpeed.get()
                    / positionCoefficient;
            config.MotionMagic.MotionMagicAcceleration = magicAccel;
            config.MotionMagic.MotionMagicCruiseVelocity = magicCruise;
            config.Slot0.kP = Shoulder.kP.get();
            config.Slot0.kI = Shoulder.kI.get();
            config.Slot0.kD = Shoulder.kD.get();
            config.Slot0.kS = Shoulder.kS.get();
            config.Slot0.kG = Shoulder.kG.get();
            config.Slot0.kV = Shoulder.kV.get();
            shoulderFR.getConfigurator().apply(config);
            shoulderFL.getConfigurator().apply(config);
            shoulderBR.getConfigurator().apply(config);
            shoulderBL.getConfigurator().apply(config);
        }, Shoulder.kP, Shoulder.kI, Shoulder.kD, Shoulder.kS, Shoulder.kG, Shoulder.kV,
                Shoulder.maxSpeed, Shoulder.maxAcceleration);
        BaseStatusSignal.refreshAll(
                shoulderAngle,
                shoulderAppliedVolts,
                shoulderSupplyCurrentAmps,
                shoulderStatorCurrentAmps,
                shoulderAngularVelocityRadPerSec,
                shoulderAngularAccelerationRadPerSecSquared,
                shoulderMotorFRTemp,
                shoulderMotorFLTemp,
                shoulderMotorBRTemp,
                shoulderMotorBLTemp);

        inputs.shoulderAngle = Rotation2d
                .fromRadians(shoulderAngle.getValueAsDouble() * positionCoefficient);

        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();

        inputs.shoulderSupplyCurrentAmps = shoulderSupplyCurrentAmps.getValueAsDouble();
        inputs.shoulderStatorCurrentAmps = shoulderStatorCurrentAmps.getValueAsDouble();
        inputs.shoulderAngularVelocityRadPerSec = shoulderAngularVelocityRadPerSec.getValueAsDouble()
                * positionCoefficient;
        inputs.shoulderAngularAccelerationRadPerSecSquared = shoulderAngularAccelerationRadPerSecSquared
                .getValueAsDouble()
                * positionCoefficient;

        inputs.shoulderFRMotorTemp = shoulderMotorFRTemp.getValueAsDouble();
        inputs.shoulderFLMotorTemp = shoulderMotorFLTemp.getValueAsDouble();
        inputs.shoulderBRMotorTemp = shoulderMotorBRTemp.getValueAsDouble();
        inputs.shoulderBLMotorTemp = shoulderMotorBLTemp.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        shoulderFR.setControl(
                motionMagicVoltage.withPosition(target.getRadians() / positionCoefficient));
    }

    @Override
    public void resetShoulderAngle(Rotation2d angle) {
        shoulderFR.setPosition(angle.getRadians() / positionCoefficient);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        shoulderFR.setControl(dutyCycleOut.withOutput(dutyCycle));
        // Only main motor needs to be controlled, others will follow
    }

    @Override
    public void setBrakeMode(boolean neutralMode) {
        shoulderFR.setNeutralMode(neutralMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        shoulderFL.setNeutralMode(neutralMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        shoulderBR.setNeutralMode(neutralMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        shoulderBL.setNeutralMode(neutralMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    @Override
    public void setCurrentLimit(int current){
        config.CurrentLimits.StatorCurrentLimit = current;
        shoulderFR.getConfigurator().apply(config);
        shoulderFL.getConfigurator().apply(config);
        shoulderBR.getConfigurator().apply(config);
        shoulderBL.getConfigurator().apply(config);
    }
    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> selfChecking = new ArrayList<>();
        selfChecking.add(new SelfCheckingTalonFX("PinkShoulderFR", shoulderFR));
        selfChecking.add(new SelfCheckingTalonFX("PinkShoulderFL", shoulderFL));
        selfChecking.add(new SelfCheckingTalonFX("PinkShoulderBR", shoulderBR));
        selfChecking.add(new SelfCheckingTalonFX("PinkShoulderBL", shoulderBL));
        return selfChecking;
    }
}
