package frc.robot.utils.selfCheck;


import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SelfCheckingLaserCAN implements SelfChecking {
	private final String label;
	private final LaserCan laserCan;

	public SelfCheckingLaserCAN(String label, LaserCan laserCan) {
		this.label = label;
		this.laserCan = laserCan;
	}

	public static String getErrorName(int status) {
		switch (status) {
		case LaserCan.LASERCAN_STATUS_NOISE_ISSUE:
			return "NOISE_ISSUE.INC_TIMING_BUDGET";
		case LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS:
			return "OUT_OF_BOUNDS";
		case LaserCan.LASERCAN_STATUS_WEAK_SIGNAL:
			return "WEAK_SIGNAL.ADJ_TIMING_AND_ROI";
		case LaserCan.LASERCAN_STATUS_WRAPAROUND:
			return "WRAP_AROUND";
		default:
			return "UNKNOWN_STATUS";
		}
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		Measurement measurement = laserCan.getMeasurement();
		if (measurement != null) {
			if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
				String statusName = getErrorName(measurement.status);
				faults.add(new SubsystemFault(
						String.format("[%s] had warning: [%s]", label, statusName),
						true));
			}
		}
		return faults;
	}

	@Override
	public Object getHardware() { return laserCan; }
}
