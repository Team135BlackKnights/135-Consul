package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingREVColorSensor;

/**
 * Creates a new color sensor on an MXP Port. This is due to the I2C port
 * causing system lockups on the RIO and RIOv2
 * 
 * @apiNote Untested
 */
public class REVColorSensorIO implements ColorSensorIO {
	private final ColorSensorV3 REVColorSensor;

	public REVColorSensorIO(int can_id) {
		this.REVColorSensor = new ColorSensorV3(Port.kMXP);
	}

	@Override
	public void updateInputs(ColorSensorIOInputs inputs) {
		inputs.colorOutput = REVColorSensor.getColor().toHexString();
		//Code below is so we get a usable value instead of just 0 to 24
		//This converts it into an int from 1 to 2048
		double output = REVColorSensor.getProximity() + 1;
		//Converts this into the sensors max range of 10cm by turning the output into a fraction of its max range and then multiplying by the max distance
		output /= 2048;
		output *= 10;
		output = 10 - output;
		inputs.proximityCentimeters = output;
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingREVColorSensor("REV_Color_Sensor", REVColorSensor));
		return hardware;
	}
}
