package frc.robot.utils.drive.Sensors;

import static org.wpilib.units.Units.Millimeters;

import java.util.ArrayList;
import java.util.List;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import org.wpilib.units.Units;
import org.wpilib.driverstation.*;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingLaserCAN;

public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
	private final LaserCan laserCan;
	private final int id;

	public DistanceSensorIOLaserCAN(int can_id) {
		this.laserCan = new LaserCan(can_id);
		this.id = can_id;
		try {
			laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
		} catch (ConfigurationFailedException e) {
			DriverStationErrors.reportError("CONFIG LASER CAN FAILED!!!",false);
		}
		
	}

	@Override
	public void updateInputs(DistanceSensorIOInputs inputs) {
		Measurement measurement = laserCan.getMeasurement();
		if (measurement != null) {
			inputs.distance = Units.Meters
					.convertFrom(measurement.distance_mm, Millimeters);
			inputs.ambientLightLevel = measurement.ambient;
			inputs.statusCode = measurement.status;
		}
	}
	@Override
	public void setRegionOfInterest(RegionOfInterest interest){
		try {
			laserCan.setRegionOfInterest(interest);
		} catch (ConfigurationFailedException e) {
			DriverStationErrors.reportError("CONFIG LASER CAN FAILED!!!",false);
		}
	}
	@Override
	public void setTimingBudget(TimingBudget budgetMS){
		try {
			laserCan.setTimingBudget(budgetMS);
		} catch (ConfigurationFailedException e) {
			DriverStationErrors.reportError("CONFIG LASER CAN FAILED!!!",false);
		}
	}
	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingLaserCAN("LASER_CAN_" + id, laserCan));
		return hardware;
	}
}
