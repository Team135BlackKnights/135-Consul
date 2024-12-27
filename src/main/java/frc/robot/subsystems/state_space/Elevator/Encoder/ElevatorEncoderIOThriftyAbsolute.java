package frc.robot.subsystems.state_space.Elevator.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIOThriftyAbsolute;

public class ElevatorEncoderIOThriftyAbsolute extends EncoderIOThriftyAbsolute implements ElevatorEncoderIO{
 public ElevatorEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRadians, isInverted);
    }
    public ElevatorEncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public ElevatorEncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public ElevatorEncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }    
}
