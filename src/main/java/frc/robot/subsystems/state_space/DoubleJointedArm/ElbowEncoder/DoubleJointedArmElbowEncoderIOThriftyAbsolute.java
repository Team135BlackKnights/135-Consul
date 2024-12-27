package frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder;

import frc.robot.utils.drive.Sensors.EncoderIOThriftyAbsolute;

public class DoubleJointedArmElbowEncoderIOThriftyAbsolute extends EncoderIOThriftyAbsolute implements DoubleJointedArmElbowEncoderIO{
 public DoubleJointedArmElbowEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRadians, isInverted);
    }
    public DoubleJointedArmElbowEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public DoubleJointedArmElbowEncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public DoubleJointedArmElbowEncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }    
}
