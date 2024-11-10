// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;

public class EncoderIOCANCoder implements EncoderIO {
    private final CANcoder encoder;
    private double conversionFactor = 1.0;
    private double encoderOffsetRadians = 0.0;
    private String name = "";
    public EncoderIOCANCoder(int canID, CANBus canBus, String name) {
        this.encoder = new CANcoder(canID, canBus);
        this.encoderOffsetRadians = Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble()) / conversionFactor;
        this.name = name;
    }

    public EncoderIOCANCoder(int canID, String name) {
        this.encoder = new CANcoder(canID);
        this.name = name;
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.absolutePositionRadians = Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble())
                / conversionFactor;
        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(encoder.getVelocity().getValueAsDouble())
                / conversionFactor;
        inputs.relativePositionRadians = (Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble())
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
		hardware.add(new SelfCheckingCANCoder(name, encoder));
		return hardware;
    }
}
