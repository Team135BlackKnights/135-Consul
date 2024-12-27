package frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder;

import frc.robot.utils.drive.Sensors.EncoderIODutyCycle;

public class DoubleJointedArmElbowEncoderIODutyCycle extends EncoderIODutyCycle implements DoubleJointedArmElbowEncoderIO {
    public DoubleJointedArmElbowEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmElbowEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmElbowEncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public DoubleJointedArmElbowEncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

}
