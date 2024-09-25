package frc.robot.utils.selfCheck.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import java.util.concurrent.ConcurrentLinkedQueue;


public class SelfCheckingCANCoder implements SelfChecking {
	private final String label;
	private final CANcoder canCoder;
	private final StatusSignal<Boolean> hardwareFaultSignal;
	private final StatusSignal<Boolean> bootEnabledSignal;
	private final StatusSignal<Boolean> badMagnetSignal;
	private final StatusSignal<Boolean> undervoltageSignal;

	public SelfCheckingCANCoder(String label, CANcoder canCoder) {
		this.label = label;
		this.canCoder = canCoder;
		this.hardwareFaultSignal = canCoder.getFault_Hardware();
		this.bootEnabledSignal = canCoder.getFault_BootDuringEnable();
		this.badMagnetSignal = canCoder.getFault_BadMagnet();
		this.undervoltageSignal = canCoder.getFault_Undervoltage();
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		if (hardwareFaultSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Hardware fault detected", label)));
		}
		if (bootEnabledSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Device booted while enabled", label)));
		}
		if (badMagnetSignal.getValue()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Magnet too weak", label)));
		}
		if (undervoltageSignal.getValue()) {
			faults.add(
					new SubsystemFault(String.format("[%s]: Under voltage", label)));
		}
		StatusSignal.refreshAll(hardwareFaultSignal, bootEnabledSignal,
				badMagnetSignal, undervoltageSignal);
		return faults;
	}

	@Override
	public Object getHardware() { return canCoder; }
}
