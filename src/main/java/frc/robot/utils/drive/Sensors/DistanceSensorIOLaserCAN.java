package frc.robot.utils.drive.Sensors;

import static edu.wpi.first.units.Units.Millimeters;

import java.util.ArrayList;
import java.util.List;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.Units;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingLaserCAN;

public class DistanceSensorIOLaserCAN implements DistanceSensorIO {
	private final LaserCan laserCan;

	public DistanceSensorIOLaserCAN(int can_id) {
		this.laserCan = new LaserCan(can_id);
	}

	@Override
	public void updateInputs(DistanceSensorIOInputs inputs) {
		Measurement measurement = laserCan.getMeasurement();
		if (measurement != null) {
			inputs.distanceMeters = Units.Meters
					.convertFrom(measurement.distance_mm, Millimeters);
			inputs.ambientLightLevel = measurement.ambient;
			inputs.statusCode = measurement.status;
		}
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingLaserCAN("LASER_CAN_0", laserCan));
		return hardware;
	}
}
