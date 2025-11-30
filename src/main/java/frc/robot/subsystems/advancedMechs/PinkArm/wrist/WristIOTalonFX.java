
package frc.robot.subsystems.advancedMechs.PinkArm.wrist;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Wrist;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
public class WristIOTalonFX implements WristIO {
    private TalonFX wrist;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0);

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<Voltage> wristVoltage;
    private final StatusSignal<Current> wristSupplyCurrent;
    private final StatusSignal<Current> wristStatorCurrent;
    private final StatusSignal<Temperature> wristTemperature;
    private final StatusSignal<AngularVelocity> wristAngularVelocity;
    private final StatusSignal<AngularAcceleration> wristAngularAcceleration;
    private TalonFXConfiguration config;
    public WristIOTalonFX() {
        wrist = new TalonFX(AdvancedMechanismConstants.PinkArm.Wrist.kMotorID,
                AdvancedMechanismConstants.PinkArm.CANBus);

         config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Wrist.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = Wrist.statorCurrentLimit;

        config.Slot0.kP =  Wrist.kP.get();

        config.Slot0.kI = Wrist.kI.get();
        config.Slot0.kD = Wrist.kD.get();

        config.Slot0.kS = Wrist.kS.get();
        config.Slot0.kV = Wrist.kV.get();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = Wrist.maxAcceleration.get() / Wrist.wristPositionCoefficient;
        config.MotionMagic.MotionMagicCruiseVelocity = Wrist.maxSpeed.get() / Wrist.wristPositionCoefficient;
        config.MotorOutput.Inverted = Wrist.inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        wrist.getConfigurator().apply(config);

        wristPosition = wrist.getRotorPosition();
        wristVoltage = wrist.getMotorVoltage();
        wristSupplyCurrent = wrist.getSupplyCurrent();
        wristStatorCurrent = wrist.getStatorCurrent();
        wristTemperature = wrist.getDeviceTemp();
        wristAngularVelocity = wrist.getRotorVelocity();
        wristAngularAcceleration = wrist.getAcceleration();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            double magicAccel = Wrist.maxAcceleration.get()
                    / Wrist.wristPositionCoefficient;
            double magicCruise = Wrist.maxSpeed.get()
                    / Wrist.wristPositionCoefficient;
            config.MotionMagic.MotionMagicAcceleration = magicAccel;
            config.MotionMagic.MotionMagicCruiseVelocity = magicCruise;
            config.Slot0.kP = Wrist.kP.get();
            config.Slot0.kI = Wrist.kI.get();
            config.Slot0.kD = Wrist.kD.get();
            config.Slot0.kS = Wrist.kS.get();
            config.Slot0.kV = Wrist.kV.get();
            wrist.getConfigurator().apply(config);
        }, Wrist.kP, Wrist.kI, Wrist.kD, Wrist.kS, Wrist.kV,
                Wrist.maxSpeed, Wrist.maxAcceleration);
         BaseStatusSignal.refreshAll(
                wristPosition,
                wristVoltage,
                wristSupplyCurrent,
                wristStatorCurrent,
                wristTemperature,
                wristAngularVelocity,
                wristAngularAcceleration);
        inputs.wristAngle =
                Rotation2d.fromRadians(wristPosition.getValueAsDouble() * Wrist.wristPositionCoefficient);

        inputs.wristAppliedVolts = wristVoltage.getValueAsDouble();
        inputs.wristSupplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
        inputs.wristStatorCurrentAmps = wristStatorCurrent.getValueAsDouble();
        inputs.wristMotorTemp = wristTemperature.getValueAsDouble();

        inputs.wristAngularVelocityRadPerSec =
                wristAngularVelocity.getValueAsDouble() * Wrist.wristPositionCoefficient;

        inputs.wristAngularAccelerationRadPerSecSquared =
                wristAngularAcceleration.getValueAsDouble() * Wrist.wristPositionCoefficient;
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        wrist.setControl(positionVoltage.withPosition(target.getRadians() / Wrist.wristPositionCoefficient));
    }

    @Override
    public void resetWristAngle(Rotation2d angle) {
        wrist.setPosition(angle.getRadians() / Wrist.wristPositionCoefficient);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        wrist.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        wrist.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    @Override
    public void setCurrentLimit(int current){
        config.CurrentLimits.StatorCurrentLimit = current;
        wrist.getConfigurator().apply(config);
    }
    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> selfChecking = new ArrayList<>();
        selfChecking.add(new SelfCheckingTalonFX("PinkWrist", wrist));
        return selfChecking;
    }
}
