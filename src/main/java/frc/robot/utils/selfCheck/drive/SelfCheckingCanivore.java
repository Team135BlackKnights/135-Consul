package frc.robot.utils.selfCheck.drive;

import java.nio.channels.UnsupportedAddressTypeException;
import java.util.concurrent.ConcurrentLinkedQueue;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;
/**
 * @apiNote Untested, needs to be checked with hardware
 * Code for a self checking CANivore through the Phoenix 6 API.
 * Does not technically ping the CANivore, instead looks at the CAN bus.
 */
public class SelfCheckingCanivore implements SelfChecking {
	private final String label;
	private final CANBusStatus busStatus;
	private final boolean isNetworkCANFD;
	

	/**
	 * Creates a new self checking Canivore (CANBus)
	 * @param label the name of the can bus
	 */
	public SelfCheckingCanivore(String label) {
		this.label = label;
		
		this.busStatus = CANBus.getStatus(label);
		this.isNetworkCANFD = CANBus.isNetworkFD(label); 
	}
	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> hardwareFaultList = new ConcurrentLinkedQueue<>();
		if (!isNetworkCANFD){
			hardwareFaultList.add(new SubsystemFault(String.format("Canivore is not CanFD ", label)));
		}
		if (busStatus.Status == StatusCode.InvalidNetwork){
			hardwareFaultList.add(new SubsystemFault(String.format("Invalid network ", label)));
		}
		if (busStatus.BusUtilization >= .9) {
			hardwareFaultList.add(new SubsystemFault(String.format("High CAN Utilization ", label)));
		}
		if (busStatus.REC > 0){
			hardwareFaultList.add(new SubsystemFault("There are "+ busStatus.REC + " recieve error(s) ", true));
		}
		if (busStatus.TEC > 0){
			hardwareFaultList.add(new SubsystemFault("There are" + busStatus.TEC + " transmit error(s) ", true));
		}
		return hardwareFaultList;
	}
	/**
	 * This should never be called, since you're trying to access the physical hardware of the CAN Bus. What're you doing? -N
	 * @return
	 */
	@Override 
	public Object getHardware(){
		throw new UnsupportedAddressTypeException();
	}

}


