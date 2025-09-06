// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.Constants.EncoderType;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Interfaces with a REV Analog Encoder.
 *
 * This class is designed for encoders connected to the SPARK data breakout
 * board
 * via the absolute pin. It is NOT intended for 6P encoders or those using the
 * absolute encoder adapter—see EncoderIOREVAbsolute for those cases.
 *
 * Note: Ensure the offset is provided in rotations, NOT radians!
 */
public class EncoderIOREVAnalog implements EncoderIO {
    private final SparkAnalogSensor encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRotations = 0.0;
    private boolean isInverted = false;

    public EncoderIOREVAnalog(SparkBase spark, double conversionFactor, double encoderOffsetRotations,
            boolean isInverted) {
        this.encoder = spark.getAnalog();
        this.conversionFactor = conversionFactor;
        this.encoderOffsetRotations = encoderOffsetRotations;
        this.isInverted = isInverted;
    }

    public EncoderIOREVAnalog(SparkBase spark, double conversionFactor, double encoderOffsetRotations) {
        this(spark, conversionFactor, encoderOffsetRotations, false);
    }

    public EncoderIOREVAnalog(SparkBase spark, double conversionFactor) {
        this(spark, conversionFactor, 0, false);
    }

    public EncoderIOREVAnalog(SparkBase spark) {
        this(spark, 1.0, 0, false);
    }

    private double getAbsoluteEncoderRad(double voltage) {
        // gets the voltage and divides by the maximum voltage to get a percent, then
        // multiplies that percent by 2pi to get a degree heading.
        double angle = voltage / RobotController.getVoltage3V3(); // use 5V when plugged into RIO 3.3V when using
                                                                  // breakout board
        angle *= 2 * Math.PI;
        // adds the offset in
        angle -= Units.rotationsToRadians(encoderOffsetRotations);
        /*
         * this line of code is here because our pid loop is set to negative pi to pi,
         * but our absolute encoders read from 0 to 2pi.
         * The line of code below basically says to add 2pi if an input is below 0 to
         * get it in the range of 0 to 2pi
         */
        angle += angle <= 0 ? 2 * Math.PI : 0;
        // subtracts pi to make the 0 to 2pi range back into pi to -pi
        angle -= Math.PI; // angle > Math.PI ? 2*Math.PI : 0;
        // if encoder is reversed multiply the input by negative one
        angle *= (isInverted ? -1 : 1);
        return angle / conversionFactor;
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = getAbsoluteEncoderRad(encoder.getVoltage());
        inputs.angularVelocityRadPerSec = getAbsoluteEncoderRad(encoder.getVelocity());
        inputs.relativePositionRadians = 0; // Not supported by REV SparkMax on breakout.
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
        inputs.encoderType = EncoderType.REV_ANALOG;
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
