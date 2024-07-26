package frc.robot.utils.selfCheck.vision;

import java.nio.channels.UnsupportedAddressTypeException;
import java.util.ArrayList;
import java.util.List;

import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.LimelightHelpers.LimelightResults;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;

public class SelfCheckingLimelight implements SelfChecking{
	private final String label;
	private Runnable checkThread = () ->
	{

	};
	
	public SelfCheckingLimelight(String label, double neuralNetworkPipelineID){
		this.label = label;

	}
		
	@Override 
	public List<SubsystemFault> checkForFaults(){
		return new ArrayList<SubsystemFault>();
	}
	/**
	 * You are trying to access the physical hardware of the Limelight, which is not possible. Why would someone need this? -N
	 */
	@Override
	public Object getHardware(){
		throw new UnsupportedAddressTypeException();
	}
}
