package frc.robot.utils.selfCheck;


import com.revrobotics.ColorSensorV3;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SelfCheckingREVColorSensor implements SelfChecking{
	private final String label;
	private final ColorSensorV3 sensor;
	public SelfCheckingREVColorSensor(String label, ColorSensorV3 sensor){
		this.label = label;
		this.sensor = sensor;
	}
	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
		if (!sensor.isConnected()){
			faults.add(new SubsystemFault(
				String.format("Disconnected ", label)));
		}
		return faults;
	}
	@Override
	public Object getHardware() {
		return sensor;
	 }
	
}

