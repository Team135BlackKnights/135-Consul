package frc.robot.utils.selfCheck.drive;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.revrobotics.CANSparkBase.FaultID;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;

public class SelfCheckingSparkBase implements SelfChecking {
	private final String label;
	private final CANSparkBase sparkBase;

	public SelfCheckingSparkBase(String label, CANSparkBase sparkBase) {
		this.label = label;
		this.sparkBase = sparkBase;
	}

	/**
	 * Untested, Undocumented code
	 * 
	 * @param faultBits all errors as a short
	 * @return list of faultID
	 */
	public static List<FaultID> getActiveFaults(short faultBits) {
		List<FaultID> activeFaults = new ArrayList<>();
		for (FaultID fault : FaultID.values()) {
			if ((faultBits & (1 << fault.value)) != 0) {
				activeFaults.add(fault);
			}
		}
		return activeFaults;
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		REVLibError errorName = sparkBase.getLastError();
		if (errorName != REVLibError.kOk) {
			faults.add(new SubsystemFault(String
					.format("[%s]: Device has REVLIBError %s", label, errorName)));
		}
		short faultBits = sparkBase.getFaults(); //why isnt this built in???
		List<FaultID> activeFaults = getActiveFaults(faultBits);
		for (FaultID fault : activeFaults) {
			faults.add(new SubsystemFault(String
					.format("[%s]: Device has FaultID %s", label, fault.name())));
		}
		short stickyFaultBits = sparkBase.getStickyFaults(); //why isnt this built in???
		List<FaultID> activeStickyFaults = getActiveFaults(stickyFaultBits);
		for (FaultID fault : activeStickyFaults) {
			faults.add(new SubsystemFault(String.format(
					"[%s]: Device has StickyFaultID %s", label, fault.name())));
		}
		return faults;
	}

	@Override
	public Object getHardware() { return sparkBase; }
}