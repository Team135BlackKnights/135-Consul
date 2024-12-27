package frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder;

import com.revrobotics.spark.SparkBase;

import frc.robot.utils.drive.Sensors.EncoderIOREVAbsolute;

public class DoubleJointedArmArmEncoderIOREVAbsolute extends EncoderIOREVAbsolute implements DoubleJointedArmArmEncoderIO {
    public DoubleJointedArmArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        super(spark, conversionFactor, encoderOffsetRotations, isInverted);
    }

    public DoubleJointedArmArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public DoubleJointedArmArmEncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public DoubleJointedArmArmEncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }
}
