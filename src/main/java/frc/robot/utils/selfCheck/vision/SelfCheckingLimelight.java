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
	private final double pipelineID;

	
	public SelfCheckingLimelight(String label, double neuralNetworkPipelineID){
		this.label = label;
		this.pipelineID = neuralNetworkPipelineID;

	}
		
	@Override 
	public List<SubsystemFault> checkForFaults(){
		ArrayList<SubsystemFault> faultList = new ArrayList<SubsystemFault>();
		Runnable checkThread = () ->
		{
			LimelightResults firstResult = LimelightHelpers.getLatestResults(label);
			double firstTimestamp = firstResult.targetingResults.timestamp_LIMELIGHT_publish;
			if (firstResult.error != ""){
				faultList.add(new SubsystemFault(String.format(firstResult.error, label)));
			}	
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {		
			}
			LimelightResults secondResult = LimelightHelpers.getLatestResults(label);
			if (secondResult.error !=""){
				faultList.add(new SubsystemFault(String.format(secondResult.error,label)));
			}
			double secondTimestamp = secondResult.targetingResults.timestamp_LIMELIGHT_publish;
			if (firstTimestamp == secondTimestamp){
				faultList.add(new SubsystemFault(String.format("Limelight disconnected", label)));
			}
			if (LimelightHelpers.getCurrentPipelineIndex(label) != pipelineID){
				faultList.add(new SubsystemFault(String.format("Wrong pipeline or coral disconnected", label)));
			}
		};
		checkThread.run();
		return faultList;
	}
	/**
	 * This is not supported, and will throw an error.
	 * You are trying to access the physical hardware of the Limelight, which is not possible. Why would someone need this? -N
	 */
	@Override
	public Object getHardware(){
		throw new UnsupportedAddressTypeException();
	}
}
