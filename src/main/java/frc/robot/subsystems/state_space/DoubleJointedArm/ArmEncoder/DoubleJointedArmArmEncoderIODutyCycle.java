package frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder;

import frc.robot.utils.drive.Sensors.EncoderIODutyCycle;

public class DoubleJointedArmArmEncoderIODutyCycle extends EncoderIODutyCycle implements DoubleJointedArmArmEncoderIO {
    public DoubleJointedArmArmEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmArmEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmArmEncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public DoubleJointedArmArmEncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

}
