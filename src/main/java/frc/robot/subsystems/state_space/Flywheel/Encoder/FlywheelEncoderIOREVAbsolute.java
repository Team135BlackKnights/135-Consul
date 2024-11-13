package frc.robot.subsystems.state_space.Flywheel.Encoder;

import com.revrobotics.spark.SparkBase;

import frc.robot.utils.drive.Sensors.EncoderIOREVAbsolute;

public class FlywheelEncoderIOREVAbsolute extends EncoderIOREVAbsolute implements FlywheelEncoderIO {
    public FlywheelEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(spark, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public FlywheelEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public FlywheelEncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public FlywheelEncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }
}
