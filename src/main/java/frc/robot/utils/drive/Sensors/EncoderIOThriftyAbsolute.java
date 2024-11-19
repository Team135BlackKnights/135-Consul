// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.Constants.EncoderType;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * This class is used to interface with a Thrifty Absolute Encoder. An example
 * would be the ThriftySwerve encoder.
 * Almost for any encoder, the conversion factor is 1.0.
 * BE SURE TO PROVIDE OFFSET IN RADIANS! NOT ROTATIONS!
 */
public class EncoderIOThriftyAbsolute implements EncoderIO {
    private final AnalogInput encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRadians = 0.0;
    private boolean isAbsoluteEncoderInverted = false;

    public EncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians,
            boolean isInverted) {
        this.encoder = new AnalogInput(rioPort);
        this.conversionFactor = conversionFactor;
        this.encoderOffsetRadians = encoderOffsetRadians;
        this.isAbsoluteEncoderInverted = isInverted;
    }
    public EncoderIOThriftyAbsolute(int rioPort, double conversionFactor, double encoderOffsetRadians) {
        this(rioPort, conversionFactor, encoderOffsetRadians, false);
    }
    public EncoderIOThriftyAbsolute(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public EncoderIOThriftyAbsolute(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        double absolutePositionPercent = encoder.getVoltage() / RobotController.getVoltage5V();
        if (isAbsoluteEncoderInverted) {
            absolutePositionPercent = 1 - absolutePositionPercent;
        }
        double currentPosition = new Rotation2d(absolutePositionPercent * 2.0 * Math.PI)
                .minus(Rotation2d.fromRadians(encoderOffsetRadians)).getRadians() / conversionFactor;
        // we set velocity before position because the position is used to calculate the
        // velocity
        inputs.angularVelocityRadPerSec = (currentPosition - inputs.absolutePositionRadians)
                / (TimeUtil.getRealTimeSeconds() - inputs.timestampSeconds);
        inputs.absolutePositionRadians = currentPosition;

        inputs.relativePositionRadians = 0;
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
        inputs.encoderType = EncoderType.THRIFTY_ABSOLUTE;
    }

    /**
     * This function only resets relative, absolute offset stays the same.
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
        conversionFactor = factor;
    }

    public List<SelfChecking> getSelfCheckingHardware() {
        List<SelfChecking> hardware = new ArrayList<SelfChecking>();
        return hardware;
    }
}
