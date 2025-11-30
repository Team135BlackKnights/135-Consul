package frc.robot.subsystems.advancedMechs.PinkArm.extension;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Extension;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class ExtensionIOTalonFX implements ExtensionIO {
    private final TalonFX extensionOne;
    private final TalonFX extensionTwo;
    Follower followControlRequest;
    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final StatusSignal<Angle> extensionPositionInMeters;
    private final StatusSignal<Voltage> extensionAppliedVolts;
    private final StatusSignal<Current> extensionSupplyCurrentAmps;
    private final StatusSignal<Current> extensionStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> extensionVelocityMetersPerSec;
    private final StatusSignal<AngularAcceleration> extensionAccelerationMetersPerSecSquared;
    private final StatusSignal<Temperature> extensionOneMotorTemp;
    private final StatusSignal<Temperature> extensionTwoMotorTemp;
    private TalonFXConfiguration config;

    public ExtensionIOTalonFX() {
        extensionOne = new TalonFX(Extension.kMotorOneID,
                PinkArm.CANBus);
        extensionTwo = new TalonFX(Extension.kMotorTwoID,
                PinkArm.CANBus);
        followControlRequest = new Follower(Extension.kMotorOneID, MotorAlignmentValue.Opposed);
        config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Extension.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Extension.statorCurrentLimit;
        config.Slot0.kP = Extension.kP.get();
        config.Slot0.kI = Extension.kI.get();
        config.Slot0.kD = Extension.kD.get();

        config.Slot0.kS = Extension.kS.get();
        config.Slot0.kV = Extension.kV.get();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.MotionMagic.MotionMagicAcceleration = Extension.maxAcceleration.get()
                / Extension.extensionPositionCoefficient;
        config.MotionMagic.MotionMagicCruiseVelocity = Extension.maxSpeed.get()
                / Extension.extensionPositionCoefficient;
        config.MotorOutput.Inverted = Extension.inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        extensionOne.getConfigurator().apply(config);
        extensionTwo.getConfigurator().apply(config);

        extensionTwo.setControl(followControlRequest);

        extensionPositionInMeters = extensionOne.getPosition();
        extensionAppliedVolts = extensionOne.getMotorVoltage();
        extensionSupplyCurrentAmps = extensionOne.getSupplyCurrent();
        extensionStatorCurrentAmps = extensionOne.getStatorCurrent();
        extensionVelocityMetersPerSec = extensionOne.getRotorVelocity();
        extensionAccelerationMetersPerSecSquared = extensionOne.getAcceleration();
        extensionOneMotorTemp = extensionOne.getDeviceTemp();
        extensionTwoMotorTemp = extensionTwo.getDeviceTemp();
    }

    @Override
    public void updateInputs(ExtensionIOInputs inputs) {
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            double magicAccel = Extension.maxAcceleration.get()
                    / Extension.extensionPositionCoefficient;
            double magicCruise = Extension.maxSpeed.get()
                    / Extension.extensionPositionCoefficient;
            config.MotionMagic.MotionMagicAcceleration = magicAccel;
            config.MotionMagic.MotionMagicCruiseVelocity = magicCruise;
            config.Slot0.kP = Extension.kP.get();
            config.Slot0.kI = Extension.kI.get();
            config.Slot0.kD = Extension.kD.get();
            config.Slot0.kS = Extension.kS.get();
            config.Slot0.kV = Extension.kV.get();
            extensionOne.getConfigurator().apply(config);
            extensionTwo.getConfigurator().apply(config);
        }, Extension.kP, Extension.kI, Extension.kD, Extension.kS, Extension.kV,
                Extension.maxSpeed, Extension.maxAcceleration);
        BaseStatusSignal.refreshAll(
                extensionPositionInMeters,
                extensionAppliedVolts,
                extensionSupplyCurrentAmps,
                extensionStatorCurrentAmps,
                extensionVelocityMetersPerSec,
                extensionAccelerationMetersPerSecSquared,
                extensionOneMotorTemp,
                extensionTwoMotorTemp);
        inputs.extensionPositionInMeters = (extensionPositionInMeters.getValueAsDouble())
                * Extension.extensionPositionCoefficient;

        inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
        inputs.extensionSupplyCurrentAmps = extensionSupplyCurrentAmps.getValueAsDouble();
        inputs.extensionStatorCurrentAmps = extensionStatorCurrentAmps.getValueAsDouble();
        inputs.extensionVelocityMetersPerSec = extensionVelocityMetersPerSec.getValueAsDouble()
                * Extension.extensionPositionCoefficient;
        inputs.extensionAccelerationMetersPerSecSquared = extensionAccelerationMetersPerSecSquared.getValueAsDouble()
                * Extension.extensionPositionCoefficient;

        inputs.extensionOneMotorTemp = extensionOneMotorTemp.getValueAsDouble();
        inputs.extensionTwoMotorTemp = extensionTwoMotorTemp.getValueAsDouble();
    }

    @Override
    public void setTargetExtension(double positionInMeters) {
        extensionOne
                .setControl(positionVoltage.withPosition(positionInMeters / Extension.extensionPositionCoefficient));
    }

    @Override
    public void resetExtensionPosition(double positionInMeters) {
        extensionOne.setPosition(positionInMeters / Extension.extensionPositionCoefficient);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        extensionOne.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        extensionTwo.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        extensionOne.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void setCurrentLimit(int current) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = current;
        extensionOne.getConfigurator().apply(config);
    }
        @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> selfChecking = new ArrayList<>();
        selfChecking.add(new SelfCheckingTalonFX("PinkExtensionOne", extensionOne));
        selfChecking.add(new SelfCheckingTalonFX("PinkExtensionTwo", extensionTwo));
        return selfChecking;
    }
}
