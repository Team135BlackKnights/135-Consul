package frc.robot.utils.selfCheck.drive;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

import java.util.concurrent.ConcurrentLinkedQueue;


public class SelfCheckingPWMMotor implements SelfChecking {
	private final String label;
	private final PWMMotorController motor;

	public SelfCheckingPWMMotor(String label, PWMMotorController motor) {
		this.label = label;
		this.motor = motor;
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		if (!motor.isAlive()) {
			faults.add(new SubsystemFault(
					String.format("[%s]: Device timed out", label)));
		}
		return faults;
	}

	@Override
	public Object getHardware() { return motor; }
}
