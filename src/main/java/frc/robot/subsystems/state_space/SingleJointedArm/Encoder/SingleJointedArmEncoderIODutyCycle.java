package frc.robot.subsystems.state_space.SingleJointedArm.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIODutyCycle;

public class SingleJointedArmEncoderIODutyCycle extends EncoderIODutyCycle implements SingleJointedArmEncoderIO {
    public SingleJointedArmEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public SingleJointedArmEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public SingleJointedArmEncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public SingleJointedArmEncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

}
