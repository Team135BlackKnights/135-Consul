package frc.robot.subsystems.state_space.Flywheel.Encoder;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.drive.Sensors.EncoderIOCANCoder;

public class FlywheelEncoderIOCANCoder extends EncoderIOCANCoder implements FlywheelEncoderIO {
    public FlywheelEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations, boolean isInverted) {
        super(canID, canBus, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public FlywheelEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations) {
        this(canID, canBus, name, conversionFactor, encoderOffsetRotations, false);
    }

    public FlywheelEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor) {
        this(canID, canBus, name, conversionFactor, 0, false);
    }

    public FlywheelEncoderIOCANCoder(int canID, CANBus canBus, String name) {
        this(canID, canBus, name, 1.0, 0, false);
    }

    public FlywheelEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(canID, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public FlywheelEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations) {
        this(canID, name, conversionFactor, encoderOffsetRotations, false);
    }

    public FlywheelEncoderIOCANCoder(int canID, String name, double conversionFactor) {
        this(canID, name, conversionFactor, 0, false);
    }

    public FlywheelEncoderIOCANCoder(int canID, String name) {
        this(canID, name, 1.0, 0, false);
    }
}
