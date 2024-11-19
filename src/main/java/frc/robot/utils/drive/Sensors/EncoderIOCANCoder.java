// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.Constants.EncoderType;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * This class is used to interface with a CANCoder.
 * Almost for any encoder, the conversion factor is 1.0.
 * BE SURE TO PROVIDE OFFSET IN ROTATIONS! NOT RADIANS!
 */
public class EncoderIOCANCoder implements EncoderIO {
    private final CANcoder encoder;
    private double conversionFactor = 1.0;
    private String name = "";
    private StatusSignal<Angle> absolutePosition;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Angle> position;

    public EncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations, boolean isInverted) {
        this.encoder = new CANcoder(canID, canBus);
        MagnetSensorConfigs sensorConfig = new MagnetSensorConfigs().withMagnetOffset(encoderOffsetRotations)
                .withSensorDirection(isInverted ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive);
        encoder.getConfigurator().apply(sensorConfig);
        absolutePosition = encoder.getAbsolutePosition();
        velocity = encoder.getVelocity();
        position = encoder.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50, absolutePosition, velocity, position);
        encoder.optimizeBusUtilization();
        this.conversionFactor = conversionFactor;
        this.name = name;
    }

    public EncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations) {
        this(canID, canBus, name, conversionFactor, encoderOffsetRotations, false);
    }

    public EncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor) {
        this(canID, canBus, name, conversionFactor, 0, false);
    }

    public EncoderIOCANCoder(int canID, CANBus canBus, String name) {
        this(canID, canBus, name, 1.0, 0, false);
    }

    public EncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        this.encoder = new CANcoder(canID);
        MagnetSensorConfigs sensorConfig = new MagnetSensorConfigs().withMagnetOffset(encoderOffsetRotations)
                .withSensorDirection(isInverted ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive);
        encoder.getConfigurator().apply(sensorConfig);
        absolutePosition = encoder.getAbsolutePosition();
        velocity = encoder.getVelocity();
        position = encoder.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50, absolutePosition, velocity, position);
        encoder.optimizeBusUtilization();
        this.name = name;
        this.conversionFactor = conversionFactor;
    }

    public EncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations) {
        this(canID, name, conversionFactor, encoderOffsetRotations, false);
    }

    public EncoderIOCANCoder(int canID, String name, double conversionFactor) {
        this(canID, name, conversionFactor, 0, false);
    }

    public EncoderIOCANCoder(int canID, String name) {
        this(canID, name, 1.0, 0, false);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        BaseStatusSignal.refreshAll(absolutePosition, velocity, position);
        inputs.absolutePositionRadians = Units.rotationsToRadians(absolutePosition.getValueAsDouble())
                / conversionFactor;
        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble())
                / conversionFactor;
        inputs.relativePositionRadians = Units
                .rotationsToRadians(position.getValueAsDouble())
                / conversionFactor;
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
        inputs.encoderType = EncoderType.CTRE;
    }

    /**
     * This function only resets relative, absolute stays the same.
     */
    @Override
    public void reset() {
        encoder.setPosition(0);
    }

    /*
     * Higher gear ratio (>1) means a reduction
     */
    @Override
    public void setGearRatio(double factor) {
        conversionFactor = factor;
    }

    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        hardware.add(new SelfCheckingCANCoder(name, encoder));
        return hardware;
    }
}
