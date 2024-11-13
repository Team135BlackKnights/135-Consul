package frc.robot.subsystems.state_space.Elevator.Encoder;

import com.revrobotics.spark.SparkBase;

import frc.robot.utils.drive.Sensors.EncoderIOREVAbsolute;

public class ElevatorEncoderIOREVAbsolute extends EncoderIOREVAbsolute implements ElevatorEncoderIO {
    public ElevatorEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(spark, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public ElevatorEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public ElevatorEncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public ElevatorEncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }
}
