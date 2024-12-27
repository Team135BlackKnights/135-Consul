package frc.robot.subsystems.state_space.Elevator.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIODutyCycle;

public class ElevatorEncoderIODutyCycle extends EncoderIODutyCycle implements ElevatorEncoderIO {
    public ElevatorEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public ElevatorEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public ElevatorEncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public ElevatorEncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

}
