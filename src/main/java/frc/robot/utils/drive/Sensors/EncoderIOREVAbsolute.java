// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

/**
 * This class is used to interface with a REV Absolute Encoder. This would be
 * used with a REV Through Bore Encoder PLUGGED INTO A SPARK MAX / FLEX.
 * Almost for any encoder, the conversion factor is 1.0.
 * BE SURE TO PROVIDE OFFSET IN ROTATIONS! NOT RADIANS!
 */
public class EncoderIOREVAbsolute implements EncoderIO {
    private final SparkAbsoluteEncoder encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRotations = 0.0;

    public EncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this.encoder = spark.getAbsoluteEncoder();
        this.conversionFactor = conversionFactor;
        this.encoderOffsetRotations = encoderOffsetRotations;
    }

    public EncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0);
    }

    public EncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = Units.rotationsToRadians(encoder.getPosition() - encoderOffsetRotations)
                / conversionFactor;
        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity())
                / conversionFactor;
        inputs.relativePositionRadians = 0; // Not supported by REV SparkMax on breakout.
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
    }

	/**
     * This function only resets relative, absolute stays the same.
     */
    @Override
    public void reset() {
        encoderOffsetRotations = 0;

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
