package frc.robot.utils.drive.Sensors;


import java.util.ArrayList;
import java.util.List;

import org.wpilib.hardware.discrete.DigitalInput;
import frc.robot.utils.selfCheck.SelfChecking;

public class DistanceSensorIOBeamBreak implements DistanceSensorIO {
	private final DigitalInput beamBreak;

	public DistanceSensorIOBeamBreak(int pwmPin) {
		this.beamBreak = new DigitalInput(pwmPin);
	}

	@Override
	public void updateInputs(DistanceSensorIOInputs inputs) {
		inputs.statusCode = 1;
		inputs.distanceMeters = beamBreak.get() ? 9999 : 0;
	}
	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		return hardware; //can't self check a beam break sensor cuz PWM pin
	}
}
