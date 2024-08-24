package frc.robot.utils.selfCheck;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.ColorSensorV3;

public class SelfCheckingREVColorSensor implements SelfChecking{
	private final String label;
	private final ColorSensorV3 sensor;
	public SelfCheckingREVColorSensor(String label, ColorSensorV3 sensor){
		this.label = label;
		this.sensor = sensor;
	}
	@Override
	public List<SubsystemFault> checkForFaults() { 
		List<SubsystemFault> faults = new ArrayList<>();
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

