package frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder;

import com.ctre.phoenix6.CANBus;

import frc.robot.utils.drive.Sensors.EncoderIOCANCoder;

public class DoubleJointedArmArmEncoderIOCANCoder extends EncoderIOCANCoder implements DoubleJointedArmArmEncoderIO {
    public DoubleJointedArmArmEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations, boolean isInverted) {
        super(canID, canBus, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor,
            double encoderOffsetRotations) {
        this(canID, canBus, name, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, CANBus canBus, String name, double conversionFactor) {
        this(canID, canBus, name, conversionFactor, 0, false);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, CANBus canBus, String name) {
        this(canID, canBus, name, 1.0, 0, false);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(canID, name, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, String name, double conversionFactor, double encoderOffsetRotations) {
        this(canID, name, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, String name, double conversionFactor) {
        this(canID, name, conversionFactor, 0, false);
    }

    public DoubleJointedArmArmEncoderIOCANCoder(int canID, String name) {
        this(canID, name, 1.0, 0, false);
    }
}
