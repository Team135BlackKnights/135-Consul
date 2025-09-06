package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.utils.selfCheck.SelfChecking;

public interface DistanceSensorIO {
	@AutoLog
	public static class DistanceSensorIOInputs {
		public double distanceMeters = 9999;
		public double ambientLightLevel = 0.0;
		public int statusCode = -1;
	}

	public default void updateInputs(DistanceSensorIOInputs inputs) {
	}
	public default void setRegionOfInterest(RegionOfInterest interest){

	}
	public default void setTimingBudget(TimingBudget budgetMS){
		
	}
	public default void setDistance(int distanceMM) {
	}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
