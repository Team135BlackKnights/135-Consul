package frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder;

import com.revrobotics.spark.SparkBase;

import frc.robot.utils.drive.Sensors.EncoderIOREVAbsolute;

public class DoubleJointedArmElbowEncoderIOREVAbsolute extends EncoderIOREVAbsolute implements DoubleJointedArmElbowEncoderIO {
    public DoubleJointedArmElbowEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(spark, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmElbowEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmElbowEncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public DoubleJointedArmElbowEncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }
}
