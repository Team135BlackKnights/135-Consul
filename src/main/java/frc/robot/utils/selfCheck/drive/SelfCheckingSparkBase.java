package frc.robot.utils.selfCheck.drive;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import com.revrobotics.REVLibError;

public class SelfCheckingSparkBase implements SelfChecking {
	private final String label;
	private final SparkBase sparkBase;

	public SelfCheckingSparkBase(String label, SparkBase sparkBase) {
		this.label = label;
		this.sparkBase = sparkBase;
	}

	private List<SubsystemFault> convertFaultsToSubsysFaults(Faults faults, boolean sticky) {
		List<SubsystemFault> activeFaults = new ArrayList<>();
		if (faults.can) {
			activeFaults.add(new SubsystemFault(String.format("[%s]: CAN bus is off (Check CAN connector!)", label),
					!sticky, sticky));
		}
		if (faults.temperature) {
			activeFaults.add(
					new SubsystemFault(String.format("[%s]: Temperature fault (too hot!)", label), !sticky, sticky));
		}
		if (faults.escEeprom) {
			activeFaults.add(new SubsystemFault(String.format("[%s]: ESC EEPROM fault ", label), !sticky, sticky));
		}
		if (faults.firmware) {
			activeFaults
					.add(new SubsystemFault(String.format("[%s]: Firmware fault (Outdated?)", label), !sticky, sticky));
		}
		if (faults.gateDriver) {
			activeFaults.add(new SubsystemFault(String
					.format("[%s]: Gate driver fault (Bad connection wires/blown internals)", label, !sticky, sticky)));
		}
		if (faults.motorType) {
			activeFaults.add(new SubsystemFault(
					String.format("[%s]: Motor type fault (Brush instead of brushless)", label), !sticky, sticky));
		}
		if (faults.sensor) {
			activeFaults.add(new SubsystemFault(
					String.format("[%s]: Sensor fault (Check sensor wire for broken)", label), !sticky, sticky));
		}
		if (faults.other) {
			activeFaults.add(new SubsystemFault(String.format("[%s]: Other fault (Unknown)", label), !sticky, sticky));
		}
		return activeFaults;
	}

	/**
	 * Get the active faults of the SparkBase, sticky and non-sticky
	 * 
	 * @return list of SubsystemFaults
	 */
	private List<SubsystemFault> getActiveFaults() {
		List<SubsystemFault> activeFaults = new ArrayList<>();
		activeFaults.addAll(convertFaultsToSubsysFaults(sparkBase.getFaults(), false));
		activeFaults.addAll(convertFaultsToSubsysFaults(sparkBase.getStickyFaults(), true));
		return activeFaults;
	}

	private List<SubsystemFault> convertWarningToSubsystemWarning(Warnings warnings, boolean sticky) {
		List<SubsystemFault> activeWarnings = new ArrayList<>();
		if (warnings.brownout) {
			activeWarnings.add(
					new SubsystemFault(String.format("[%s]: Brownout detected (Low Voltage)", label), !sticky, sticky));
		}
		if (warnings.escEeprom) {
			activeWarnings.add(
					new SubsystemFault(String.format("[%s]: Internal ESC EEPROM warning", label), !sticky, sticky));
		}
		if (warnings.extEeprom) {
			activeWarnings
					.add(new SubsystemFault(String.format("[%s]: External EEPROM warning", label), !sticky, sticky));
		}
		if (warnings.hasReset) {
			activeWarnings
					.add(new SubsystemFault(String.format("[%s]: Has reset (lost power)", label), !sticky, sticky));
		}
		if (warnings.overcurrent) {
			activeWarnings
					.add(new SubsystemFault(String.format("[%s]: Over current detected", label), !sticky, sticky));
		}
		if (warnings.sensor) {
			activeWarnings.add(new SubsystemFault(
					String.format("[%s]: Temporary sensor issue (Lost connection?)", label), !sticky, sticky));
		}
		if (warnings.stall) {
			activeWarnings.add(
					new SubsystemFault(String.format("[%s]: Stall detected (Motor is stuck)", label), !sticky, sticky));
		}
		if (warnings.other) {
			activeWarnings
					.add(new SubsystemFault(String.format("[%s]: Other warning (Unknown)", label), !sticky, sticky));
		}
		return activeWarnings;
	}

	private List<SubsystemFault> getActiveWarnings() {
		List<SubsystemFault> activeWarnings = new ArrayList<>();
		activeWarnings.addAll(convertWarningToSubsystemWarning(sparkBase.getWarnings(), false));
		activeWarnings.addAll(convertWarningToSubsystemWarning(sparkBase.getStickyWarnings(), true));
		return activeWarnings;
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		REVLibError errorName = sparkBase.getLastError();
		if (errorName != REVLibError.kOk) {
			faults.add(new SubsystemFault(String
					.format("[%s]: failed to get last error, REVLIBError %s", label, errorName)));
		}
		if (sparkBase.hasActiveFault() || sparkBase.hasStickyFault()) {
			faults.addAll(getActiveFaults());
		}
		if (sparkBase.hasActiveWarning() || sparkBase.hasStickyWarning()) {
			faults.addAll(getActiveWarnings());
		}
		// Clear the sticky faults / warnings ON the motor
		REVLibError clearError = sparkBase.clearFaults();
		if (clearError != REVLibError.kOk) {
			faults.add(new SubsystemFault(String
					.format("[%s]: failed to clear stickys, REVLIBError %s", label, clearError)));
		}
		return faults;
	}

	@Override
	public Object getHardware() {
		return sparkBase;
	}
}