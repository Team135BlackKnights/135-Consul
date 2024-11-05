package frc.robot.utils.selfCheck.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;


import java.util.concurrent.ConcurrentLinkedQueue;


public class SelfCheckingREVABS implements SelfChecking{
	private final String label;
	private final AbsoluteEncoder revAbs;

	public SelfCheckingREVABS(String label, AbsoluteEncoder revAbs){
		this.label = label;
		this.revAbs = revAbs;
	}

@Override
public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
	ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
	double oldConversionFactor = revAbs.getVelocityConversionFactor();
REVLibError error = revAbs.setVelocityConversionFactor(1);
	if (error != REVLibError.kOk) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Hardware fault detected: " + error.toString() , label)));
		}
		//set to old
	REVLibError resetError = revAbs.setVelocityConversionFactor(oldConversionFactor);
	if (resetError != REVLibError.kOk) {
		faults.add(new SubsystemFault(
				String.format("[%s]: Hardware fault detected: " + resetError.toString(), label)));
	}
	return faults;
}
@Override
public Object getHardware() {
	return revAbs;
 }
	}
