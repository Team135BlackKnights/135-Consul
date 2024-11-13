package frc.robot.subsystems.state_space.SingleJointedArm.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIOThriftyAbsolute;

public class SingleJointedArmEncoderIOThriftyAbsolute extends EncoderIOThriftyAbsolute implements SingleJointedArmEncoderIO{
 public SingleJointedArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRadians, isInverted);
    }
    public SingleJointedArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public SingleJointedArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public SingleJointedArmEncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }    
}
