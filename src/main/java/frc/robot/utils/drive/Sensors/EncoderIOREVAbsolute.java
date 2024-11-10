// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

public class EncoderIOREVAbsolute implements EncoderIO {
 private double conversionFactor = 1.0;
  private double encoderOffsetRadians = 0.0;
  private final SparkBase spark;
  private final boolean isInverted;
  private final AbsoluteEncoder encoder;

    public EncoderIOREVAbsolute(SparkBase spark, boolean isInverted) {
      this.isInverted = isInverted;
    this.spark = spark;
    AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig().inverted(isInverted)
        .positionConversionFactor(1 / conversionFactor)
        .velocityConversionFactor(1 /conversionFactor);
    this.encoder = spark.getAbsoluteEncoder();
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
    encoderOffsetRadians = encoder.getPosition();
    }


    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = Units.rotationsToRadians(encoder.getPosition())
                / conversionFactor;
        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity())
                / conversionFactor;
        inputs.relativePositionRadians = (Units.rotationsToRadians(encoder.getPosition())
                / conversionFactor) - encoderOffsetRadians;
        inputs.timestampSeconds = Logger.getTimestamp() * 1e6;
    }

	/**
     * This function only resets relative, absolute stays the same.
     */
    @Override
    public void reset() {
        encoderOffsetRadians = 0;

    }

    /*
     * Higher gear ratio (>1) means a reduction
     */
    @Override
    public void setGearRatio(double factor) {
        AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig().inverted(isInverted)
        .positionConversionFactor(1 / factor)
        .velocityConversionFactor(1 /factor);
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
    encoderOffsetRadians = encoder.getPosition();
    
    }

    public List<SelfChecking> getSelfCheckingHardware() {
        		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
            return hardware;
    }
}
