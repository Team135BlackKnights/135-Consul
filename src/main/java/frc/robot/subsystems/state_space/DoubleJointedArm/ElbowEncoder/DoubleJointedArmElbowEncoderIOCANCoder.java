package frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.drive.Sensors.EncoderIOCANCoder;

public class DoubleJointedArmElbowEncoderIOCANCoder extends EncoderIOCANCoder implements DoubleJointedArmElbowEncoderIO {
    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations, boolean isInverted) {
        super(canID, canBus, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations) {
        this(canID, canBus, name, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor) {
        this(canID, canBus, name, conversionFactor, 0, false);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, CANBus canBus, String name) {
        this(canID, canBus, name, 1.0, 0, false);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(canID, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations) {
        this(canID, name, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, String name, double conversionFactor) {
        this(canID, name, conversionFactor, 0, false);
    }

    public DoubleJointedArmElbowEncoderIOCANCoder(int canID, String name) {
        this(canID, name, 1.0, 0, false);
    }
}
