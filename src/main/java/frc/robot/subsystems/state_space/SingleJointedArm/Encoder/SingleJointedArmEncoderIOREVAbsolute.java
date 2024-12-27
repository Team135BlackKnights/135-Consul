package frc.robot.subsystems.state_space.SingleJointedArm.Encoder;

import com.revrobotics.spark.SparkBase;

import frc.robot.utils.drive.Sensors.EncoderIOREVAbsolute;

public class SingleJointedArmEncoderIOREVAbsolute extends EncoderIOREVAbsolute implements SingleJointedArmEncoderIO {
    public SingleJointedArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(spark, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public SingleJointedArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public SingleJointedArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public SingleJointedArmEncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }
}
