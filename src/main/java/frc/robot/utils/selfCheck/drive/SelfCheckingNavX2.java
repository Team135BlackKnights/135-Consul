package frc.robot.utils.selfCheck.drive;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import java.util.concurrent.ConcurrentLinkedQueue;


public class SelfCheckingNavX2 implements SelfChecking {
	private final String label;
	private final AHRS navX;

	public SelfCheckingNavX2(String label, AHRS navX) {
		this.label = label;
		this.navX = navX;
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		if (!navX.isConnected()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: No communication with device", label)));
		}
		if (navX.isMagneticDisturbance() && navX.isMagnetometerCalibrated()) {
			faults.add(new SubsystemFault(String.format(
					"[%s]: Magnetic disturbance detected, or not calibrated.",
					label)));
		}
		return faults;
	}

	@Override
	public Object getHardware() { return navX; }
}
