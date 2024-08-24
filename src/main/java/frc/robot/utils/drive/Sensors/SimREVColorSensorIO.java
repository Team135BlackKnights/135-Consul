package frc.robot.utils.drive.Sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.util.Color;

public class SimREVColorSensorIO implements ColorSensorIO {
	private ColorSensorV3 REVColorSensor;
	private SimDouble red, blue, green, IR, proximity;

	public SimREVColorSensorIO(int can_id) {
		this.REVColorSensor = new ColorSensorV3(Port.kMXP);
		SimDeviceSim device = new SimDeviceSim("REV Color Sensor V3",
				Port.kMXP.value, 0x52);
		this.red = device.getDouble("Red");
		this.green = device.getDouble("Green");
		this.blue = device.getDouble("Blue");
		this.IR = device.getDouble("IR");
		this.proximity = device.getDouble("Proximity");
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
	public void setSimColor(Color color) {
		this.red.set(color.red);
		this.green.set(color.green);
		this.blue.set(color.blue);
	}

	@Override
	public void setIR(double IR, double proximity) {
		this.IR.set(IR);
		this.proximity.set(proximity);
	}
}
