package frc.robot.utils.drive.Sensors;


import java.util.ArrayList;
import java.util.List;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.DigitalInput;
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
	public void setRegionOfInterest(RegionOfInterest interest){
		// Do nothing
	}
	@Override
	public void setTimingBudget(TimingBudget budgetMS){
		// Do nothing
	}
	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		return hardware; //can't self check a beam break sensor cuz PWM pin
	}
}
