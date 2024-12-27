package frc.robot.subsystems.state_space.Flywheel.Encoder;

import frc.robot.utils.drive.Sensors.EncoderIODutyCycle;

public class FlywheelEncoderIODutyCycle extends EncoderIODutyCycle implements FlywheelEncoderIO {
    public FlywheelEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(rioPort, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public FlywheelEncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public FlywheelEncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public FlywheelEncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

}
