package frc.robot.utils.selfCheck;

import java.util.concurrent.ConcurrentLinkedQueue;

public interface SelfChecking {
	ConcurrentLinkedQueue<SubsystemFault> checkForFaults();

	Object getHardware();
}
