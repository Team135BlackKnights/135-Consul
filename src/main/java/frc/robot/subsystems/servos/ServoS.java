package frc.robot.subsystems.servos;

import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.servos.ServoConstantContainer;
import frc.robot.utils.servos.ServoPackage;
import frc.robot.utils.servos.ServoConstantContainer.ServoNames;
import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;

import java.util.List;
import java.util.Map;
import java.util.Collections;
import java.util.HashMap;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ServoS extends SubsystemChecker {
	private ServoPackage[] servoPackages = {
			new ServoPackage(ServoConstantContainer.leftServoPWMPort,
					SimServoMode.INRANGE, ServoType.REVSmartServo, 0, .02,
					ServoConstantContainer.leftLowerServoBound,
					ServoConstantContainer.leftUpperServoBound),
			new ServoPackage(ServoConstantContainer.rightServoPWMPort,
					SimServoMode.INRANGE, ServoType.REVSmartServo, -45, .02,
					ServoConstantContainer.rightLowerServoBound,
					ServoConstantContainer.rightUpperServoBound)
	};

	/**
	 * Set the servo to a specified angle
	 * 
	 * @param degrees the desired angle (in degrees)
	 */
	public void setServoDegrees(double degrees, ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			servoPackages[0].setServoDegrees(degrees);
			break;
		case rightServo:
			servoPackages[1].setServoDegrees(degrees);
			break;
		}
	}

	/**
	 * Sets the servo to a certain percent. If it's continuous, sets a POSITION
	 * from 0 to 1, where 0 is maximum left and 1 is maximum right If it's in
	 * range, set it to a constant PERCENTAGE of the max velocity
	 * 
	 * @param percent the percent to set the servo to (-1 to 1) if the servo is
	 *                   continuous, the position to be set to from (0 to 1.0) if
	 *                   it is in range mode
	 */
	public void setServoPercent(double percent, ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			servoPackages[0].setServoPercent(percent);
			break;
		case rightServo:
			servoPackages[1].setServoPercent(percent);
			break;
		}
	}

	/**
	 * @return the angular position of the servo in degrees
	 */
	public double getServoDegrees(ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			return servoPackages[0].getServoDegrees();
		case rightServo:
			return servoPackages[1].getServoDegrees();
		default:
			return 0;
		}
	}

	@Override
	public void periodic() {
		for (ServoPackage servo : servoPackages) {
			servo.updateServoSim();
		}
		SmartDashboard.putNumber("current pos LEFT", getServoDegrees(ServoNames.leftServo));
		SmartDashboard.putNumber("current pos RIGHT", getServoDegrees(ServoNames.rightServo));
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() { 
		return Collections.emptyList();
	 }

	@Override
	protected Command systemCheckCommand() { 
		return Commands.sequence(run(() -> {
			setServoDegrees(45, ServoNames.leftServo);
			setServoDegrees(-60, ServoNames.rightServo);
		}).withTimeout(1),runOnce(() ->{
			if (Math.abs(getServoDegrees(ServoNames.leftServo)-45) > 10){
				addFault("[System Check] Angle position off greater than 10 degrees for left servo. Wanted 45, got " + getServoDegrees(ServoNames.leftServo), false,true);
			}
			if (Math.abs(getServoDegrees(ServoNames.rightServo)+60) > 10){
				addFault("[System Check] Angle position off greater than 10 degrees for right servo. Wanted 60, got " + getServoDegrees(ServoNames.rightServo), false,true);
			}
		}),run(() -> {
			setServoDegrees(0, ServoNames.leftServo);
			setServoDegrees(0, ServoNames.rightServo);
		}).withTimeout(1),runOnce(() ->{
			if (Math.abs(getServoDegrees(ServoNames.leftServo)) > 10){
				addFault("[System Check] Angle position off greater than 10 degrees for left servo. Wanted 0, got " + getServoDegrees(ServoNames.leftServo), false,true);
			}
			if (Math.abs(getServoDegrees(ServoNames.rightServo)) > 10){
				addFault("[System Check] Angle position off greater than 10 degrees for right servo. Wanted 0, got " + getServoDegrees(ServoNames.rightServo), false,true);
			}
		}));
	 }

	@Override
	public double getCurrent() { return 0; } //negligible current draw

	@Override
	public HashMap<String, Double> getTemps() {
		return new HashMap<>(Map.of("NULL", 0.0));
  }

	@Override
	public void setCurrentLimit(int amps) { return; }
}
