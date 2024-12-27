package frc.robot.subsystems.state_space.Flywheel.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIOThriftyAbsolute;

public class FlywheelEncoderIOThriftyAbsolute extends EncoderIOThriftyAbsolute implements FlywheelEncoderIO{
 public FlywheelEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRadians, isInverted);
    }
    public FlywheelEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public FlywheelEncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public FlywheelEncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }    
}
