package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.ArrayList;
import java.util.List;

public class OrchestraC extends Command {
	private Orchestra orchestra;
	private String filename;

	public OrchestraC(String fileName) {
		List<ParentDevice> allDevices = new ArrayList<>();
		allDevices.addAll(RobotContainer.getOrchestraDevices());
		
		orchestra = new Orchestra();
		boolean flip =false;
		for (ParentDevice device : allDevices){
			if (flip){
				orchestra.addInstrument(device,1);
			}else{
				orchestra.addInstrument(device,0);
			}
			flip = !flip;
		}
		this.filename = fileName;
		if (Constants.currentMode == Constants.Mode.SIM) {
			System.out.println(orchestra
					.loadMusic("src/main/deploy/orchestra/" + fileName + ".chrp"));
		} else {
			System.out
					.println(orchestra.loadMusic("orchestra/" + fileName + ".chrp"));
		}
		addRequirements(RobotContainer.getAllSubsystems()); //add all orchestra groups!
	}

	@Override
	public void initialize() {
		orchestra.stop();
		orchestra.play();
		System.err.println("Is playing song? " + orchestra.isPlaying()
				+ "\nPlaying Song: " + filename);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() { return !orchestra.isPlaying(); }

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			orchestra.stop();
		}
	}

	@Override
	public boolean runsWhenDisabled() { return true; }
}
