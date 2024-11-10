package frc.robot.utils.drive.Sensors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkAnalogSensor;

public class EncoderIOSparkAnalog implements EncoderIO {
  private final double conversionFactor = 1.0;
  private final double voltageIn = 3.3;
  private double encoderOffsetRadians = 0.0;
  private final SparkBase spark;
  private final boolean isInverted;
  private final SparkAnalogSensor sparkAnalogSensor;

  public EncoderIOSparkAnalog(SparkBase spark, boolean isInverted) {
    this.isInverted = isInverted;
    this.spark = spark;
    sparkAnalogSensor = spark.getAnalog();
    AnalogSensorConfig absoluteEncoderConfig = new AnalogSensorConfig().inverted(isInverted)
    .positionConversionFactor(1 / voltageIn * 2 * Math.PI / conversionFactor)
    .velocityConversionFactor(1 / voltageIn * 2 * Math.PI / conversionFactor);
    if (spark.getClass().getName() == "com.revrobotics.spark.SparkMax") {
      SparkMaxConfig config = new SparkMaxConfig();
      config.apply(absoluteEncoderConfig);
      spark.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    } else {
      SparkFlexConfig config = new SparkFlexConfig();
      config.apply(absoluteEncoderConfig);
      spark.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
    encoderOffsetRadians = sparkAnalogSensor.getPosition();
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    inputs.absolutePositionRadians = sparkAnalogSensor.getPosition();
    inputs.angularVelocityRadPerSec = sparkAnalogSensor.getVelocity();
    inputs.relativePositionRadians = sparkAnalogSensor.getPosition() - encoderOffsetRadians;
    inputs.timestampSeconds = Logger.getTimestamp();
  }

  /**
   * This function only resets relative, absolute stays the same.
   */
  @Override
  public void reset() {
    encoderOffsetRadians = 0.0;
  }

  /*
   * Higher gear ratio (>1) means a reduction
   */
  public void setGearRatio(double factor) {
    AnalogSensorConfig absoluteEncoderConfig = new AnalogSensorConfig().inverted(isInverted)
        .positionConversionFactor(1 / voltageIn * 2 * Math.PI / factor)
        .velocityConversionFactor(1 / voltageIn * 2 * Math.PI / factor);
    if (spark.getClass().getName() == "com.revrobotics.spark.SparkMax") {
      SparkMaxConfig config = new SparkMaxConfig();
      config.apply(absoluteEncoderConfig);
      spark.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    } else {
      SparkFlexConfig config = new SparkFlexConfig();
      config.apply(absoluteEncoderConfig);
      spark.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
    encoderOffsetRadians = sparkAnalogSensor.getPosition();
  }

  public List<SelfChecking> getSelfCheckingHardware() {
    return new ArrayList<SelfChecking>();
  }
}
