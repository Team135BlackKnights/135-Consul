package frc.robot.utils.drive.Sensors;


import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.simulation.MockLaserCan;

public class DistanceSensorIOSim implements DistanceSensorIO {
	private final MockLaserCan laserCan;

	public DistanceSensorIOSim() {
		laserCan = new MockLaserCan();
		laserCan.setMeasurementPartialSim(LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT, 9999, 1);
	}

	@Override
	public void updateInputs(DistanceSensorIOInputs inputs) {
		Measurement measurement = laserCan.getMeasurement();
		if (measurement != null) {
			inputs.distanceMeters = measurement.distance_mm;
			inputs.ambientLightLevel = measurement.ambient;
			inputs.statusCode = measurement.status;
		}
	}

	@Override
	public void setDistance(int distanceMM) {
		laserCan.setMeasurementPartialSim(LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT, distanceMM, 1);
	}
}
