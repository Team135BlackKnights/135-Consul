package frc.robot.utils.selfCheck.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import java.util.concurrent.ConcurrentLinkedQueue;


public class SelfCheckingTalonFX implements SelfChecking {
	private final String label;
	private final TalonFX talon;
	private final StatusSignal<Integer> firmwareVersionSignal;
	private final StatusSignal<Boolean> hardwareFaultSignal;
	private final StatusSignal<Boolean> bootEnabledSignal;
	private final StatusSignal<Boolean> deviceTempSignal;
	private final StatusSignal<Boolean> procTempSignal;

	public SelfCheckingTalonFX(String label, TalonFX talon) {
		this.label = label;
		this.talon = talon;
		firmwareVersionSignal = talon.getVersion();
		hardwareFaultSignal = talon.getFault_Hardware();
		bootEnabledSignal = talon.getFault_BootDuringEnable();
		deviceTempSignal = talon.getFault_DeviceTemp();
		procTempSignal = talon.getFault_ProcTemp();
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		//    if (firmwareVersionSignal.getStatus() != StatusCode.OK) {
		//      faults.add(new SubsystemFault(String.format("[%s]: No communication with device",
		// label)));
		//    }
		if (hardwareFaultSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Hardware fault detected", label)));
		}
		if (bootEnabledSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Device booted while enabled", label)));
		}
		if (deviceTempSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Device temperature too high", label),
					true));
		}
		if (procTempSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Processor temperature too high", label),
					true));
		}
		StatusSignal.refreshAll(firmwareVersionSignal, hardwareFaultSignal,
				bootEnabledSignal, deviceTempSignal, procTempSignal);
		return faults;
	}

	@Override
	public Object getHardware() { return talon; }
}
