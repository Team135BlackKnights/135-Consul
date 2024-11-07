// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {

	@AutoLog
	public static class EncoderIOInputs {
        public double timestampSeconds = 0.0;
		public double relativePositionRadians = 0.0;
		public double absolutePositionRadians = 0.0;
		public double angularVelocityRadPerSec = 0.0;
	}

	public default void updateInputs(EncoderIOInputs inputs) {}

	public default void reset() {}
    /*
     * Higher gear ratio (>1) means a reduction
     */
    public default void setGearRatio(double factor) {

    }

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
