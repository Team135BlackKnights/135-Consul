// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.Constants.EncoderType;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * This class is used to interface with a duty cycle encoder. An example would
 * be the REV Through Bore Encoder using the black, red, and white wires, NOT
 * plugged into the Spark. (The
 * ratio would be 1, as it is in rotations already)
 * Almost for any encoder, the conversion factor is 1.0.
 * BE SURE TO PROVIDE OFFSET IN ROTATIONS! NOT RADIANS!
 */
public class EncoderIODutyCycle implements EncoderIO {
    private final DutyCycleEncoder encoder;
    private double conversionFactor = 1.0; // from rotations to radians
    private double encoderOffsetRotations = 0.0; // offset in rotations

    public EncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations, boolean isInverted) {
        this.encoder = new DutyCycleEncoder(rioPort);
        this.conversionFactor = conversionFactor;
        encoder.setInverted(isInverted);
        this.encoderOffsetRotations = encoderOffsetRotations;
    }

    public EncoderIODutyCycle(int rioPort, double conversionFactor, double encoderOffsetRotations) {
        this(rioPort, conversionFactor, encoderOffsetRotations, false);
    }

    public EncoderIODutyCycle(int rioPort, double conversionFactor) {
        this(rioPort, conversionFactor, 0, false);
    }

    public EncoderIODutyCycle(int rioPort) {
        this(rioPort, 1.0, 0, false);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        double encoderValue = encoder.get();
        //if (encoderValue < .88){
        //    encoderValue++;
       // }
        inputs.rawOutput = encoderValue;
        double currentPosition = Units.rotationsToRadians(encoderValue - encoderOffsetRotations)
                / conversionFactor;
       // if (currentPosition > 2.4){
        //    currentPosition -= 2*Math.PI;
       // }
        // we set velocity before position because the position is used to calculate the
        // velocity
        inputs.angularVelocityRadPerSec = (currentPosition - inputs.absolutePositionRadians)
                / (TimeUtil.getRealTimeSeconds() - inputs.timestampSeconds);
        inputs.absolutePositionRadians = currentPosition;
        inputs.relativePositionRadians = 0; // Not supported by DutyCycleEncoder.
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
        inputs.encoderType = EncoderType.DUTY_CYCLE;
    }

    /**
     * This function only resets relative, absolute offset stays the same.
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
