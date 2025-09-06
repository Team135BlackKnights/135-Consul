// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.Constants.EncoderType;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.ArrayList;
import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.util.Units;

/**
 * This class is used to interface with a REV Absolute Encoder. This would be
 * used with a REV Through Bore Encoder PLUGGED INTO A SPARK MAX / FLEX USING THE ABSOLUTE ENCODER ADAPTER {https://www.revrobotics.com/rev-11-3326/}.
 * Almost for any encoder, the conversion factor is 1.0.
 * BE SURE TO PROVIDE OFFSET IN ROTATIONS! NOT RADIANS!
 */
public class EncoderIOREVAbsolute implements EncoderIO {
    private final AbsoluteEncoder encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRotations = 0.0;
    private boolean isInverted = false;

    public EncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        this.encoder = spark.getAbsoluteEncoder();
        this.conversionFactor = conversionFactor;
        this.encoderOffsetRotations = encoderOffsetRotations;
        this.isInverted = isInverted;
    }

    public EncoderIOREVAbsolute(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public EncoderIOREVAbsolute(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public EncoderIOREVAbsolute(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = (Units.rotationsToRadians(encoder.getPosition() - encoderOffsetRotations)
                / conversionFactor) * (isInverted ? -1 : 1);
        inputs.angularVelocityRadPerSec = (Units.rotationsToRadians(encoder.getVelocity())
                / conversionFactor) * (isInverted ? -1 : 1);
        inputs.relativePositionRadians = 0; // Not supported by REV SparkMax on breakout.
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
        inputs.encoderType = EncoderType.REV_ABSOLUTE;
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
        conversionFactor = factor;
    }

    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        return hardware;
    }
}
