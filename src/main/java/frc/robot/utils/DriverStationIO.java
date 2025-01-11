// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface DriverStationIO {
	public static enum DSLEDMode{
		
	}
	@AutoLog
	public static class DriverStationIOInputs {

	}

	//public default void updateInputs(GyroIOInputs inputs) {
//
//	}

	public default void reset() {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
