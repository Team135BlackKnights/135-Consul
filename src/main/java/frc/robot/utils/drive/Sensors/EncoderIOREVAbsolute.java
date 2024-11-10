// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.util.Units;

public class EncoderIOREVAbsolute implements EncoderIO {
    private final SparkAbsoluteEncoder encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRadians = 0.0;
    public EncoderIOREVAbsolute(SparkBase spark) {
        this.encoder = spark.getAbsoluteEncoder();
        this.encoderOffsetRadians = Units.rotationsToRadians(encoder.getPosition()) / conversionFactor;
    }


    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = Units.rotationsToRadians(encoder.getPosition())
                / conversionFactor;
        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity())
                / conversionFactor;
        inputs.relativePositionRadians = (Units.rotationsToRadians(encoder.getPosition())
                / conversionFactor) - encoderOffsetRadians;
        inputs.timestampSeconds = TimeUtil.getRealTimeSeconds();
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
