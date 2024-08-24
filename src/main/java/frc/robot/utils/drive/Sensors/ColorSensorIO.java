package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.selfCheck.SelfChecking;

public interface ColorSensorIO {
		@AutoLog
	
	public static class ColorSensorIOInputs{
		public String colorOutput;
		public double proximityCentimeters;
	}

	public default void updateInputs(ColorSensorIOInputs inputs) {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
	/**
	 * Sets the SIM ONLY color of the sensor
	 */
	public default void setSimColor(Color color) {throw new UnsupportedOperationException("This sensor does not support setting the color");}
	/**
	 * Sets the SIM ONLY IR and Proximity of the sensor
	 */
	public default void setIR(double IR, double proximity) {throw new UnsupportedOperationException("This sensor does not support setting the IR and Proximity");}
	
}

