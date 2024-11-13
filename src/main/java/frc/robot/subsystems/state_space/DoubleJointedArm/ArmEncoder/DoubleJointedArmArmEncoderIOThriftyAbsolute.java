package frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder;

import frc.robot.utils.drive.Sensors.EncoderIOThriftyAbsolute;

public class DoubleJointedArmArmEncoderIOThriftyAbsolute extends EncoderIOThriftyAbsolute implements DoubleJointedArmArmEncoderIO{
 public DoubleJointedArmArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRadians, isInverted);
    }
    public DoubleJointedArmArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public DoubleJointedArmArmEncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public DoubleJointedArmArmEncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }    
}
