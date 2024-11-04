// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// Modified by 5516 "IRON MAPLE", original source:
// https://github.com/Shenzhen-Robotics-Alliance/maple-sim
package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation;
import frc.robot.utils.drive.DriveConstants;

import java.util.Arrays;

/**
 * Wrapper class around {@link SwerveModuleSimulation} that implements ModuleIO
 */
public class ModuleIOSim implements ModuleIO {
	private final SwerveModuleSimulation moduleSimulation;
	private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);
	private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);

	public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
		this.moduleSimulation = moduleSimulation;
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRads = moduleSimulation
				.getDriveWheelFinalPositionRad();
		inputs.driveVelocityRadsPerSec = moduleSimulation
				.getDriveWheelFinalSpeedRadPerSec();
		inputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
		inputs.driveSupplyCurrentAmps = Math
				.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps());
		inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
		inputs.turnPosition = Rotation2d
				.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
		inputs.turnVelocityRadsPerSec = moduleSimulation
				.getSteerRelativeEncoderSpeedRadPerSec();
		inputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
		inputs.turnSupplyCurrentAmps = Math
				.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps());
		//inputs.odoometryTimeS = OdometryTimeStampsSim.getTimeStamps();
		inputs.odometryDrivePositionsMeters = Arrays
				.stream(moduleSimulation.getCachedDriveWheelFinalPositionsRad())
				.map(position -> position
						* DriveConstants.TrainConstants.kWheelDiameter / 2)
				.toArray();
		inputs.odometryTurnPositions = Arrays
				.stream(moduleSimulation.getCachedSteerRelativeEncoderPositions())
				.mapToObj(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
	}

	@Override
	public void runDriveVolts(double volts) {
		moduleSimulation.requestDriveVoltageOut(volts);
	}

	@Override
	public void runTurnVolts(double volts) {
		moduleSimulation.requestSteerVoltageOut(volts);
	}

	@Override
	public void runCharacterization(double input) { runDriveVolts(input); }

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		runDriveVolts(MathUtil.clamp(driveFeedback.calculate(
				moduleSimulation.getDriveWheelFinalSpeedRadPerSec(),
				velocityRadsPerSec) + feedForward,-12,12));
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		double currentAngle = moduleSimulation.getSteerAbsoluteFacing().getRadians();
		double difference = angleRads - currentAngle;
		if (difference > Math.PI) {
			angleRads -= 2 * Math.PI;
		} else if (difference < -Math.PI) {
			angleRads += 2 * Math.PI;
		}
		runTurnVolts(turnFeedback.calculate(currentAngle, angleRads));
	}
	@Override
	public void setDrivePID(double kP, double kI, double kD) {
		driveFeedback.setPID(kP, kI, kD);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD, double kS) {
		turnFeedback.setPID(kP, kI, kD);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		moduleSimulation.setSteerMotorBrakeMode(enable);
	}

	@Override
	public void stop() {
		runDriveVolts(0.0);
		runTurnVolts(0.0);
	}
}