package frc.robot.utils.drive.Sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation.OdometryTimeStampsSim;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.GyroSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs;

public class GyroIOSim implements GyroIO {
	public final GyroPhysicsSimulationResults gyroPhysicsSimulationResults = new GyroPhysicsSimulationResults();
	public double previousAngularVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec;
	public Rotation2d currentGyroDriftAmount = new Rotation2d();
	private double lastGForce;
	private final GyroSimulation gyroSimulation;

	public GyroIOSim(GyroSimulation gyroSimulation) {
		this.gyroSimulation = gyroSimulation;
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = true;
		Rotation2d[] odometryYawPositions = gyroSimulation.getCachedGyroReadings();
		for (int i = 0; i < odometryYawPositions.length; i++) {
			odometryYawPositions[i] = odometryYawPositions[i].plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		}
		inputs.odometryYawPositions = odometryYawPositions;
		inputs.odometryYawTimestamps = OdometryTimeStampsSim.getTimeStamps();
		inputs.yawPosition = gyroSimulation.getGyroReading().plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		inputs.yawVelocityRadPerSec = gyroSimulation
				.getMeasuredAngularVelocityRadPerSec();
		double currentGForce = gyroPhysicsSimulationResults.gForce;
		if (Math.abs(currentGForce
				- lastGForce) > DriveConstants.RobotPhysicsSimulationConfigs.MAX_FAKE_G) {
			inputs.collisionDetected = true;
		} else {
			inputs.collisionDetected = false;
		}
		lastGForce = currentGForce;
		Logger.recordOutput("Drive/Gyro/robot true yaw (deg)",
				gyroPhysicsSimulationResults.odometryYawPositions[gyroPhysicsSimulationResults.odometryYawPositions.length
						- 1].getDegrees());
		Logger.recordOutput("Drive/Gyro/imu total drift (Deg)",
				currentGyroDriftAmount.getDegrees());
		Logger.recordOutput("Drive/Gyro/gyro reading yaw (Deg)",
				inputs.yawPosition.getDegrees());
		Logger.recordOutput("Drive/Gyro/angular velocity (Deg per Sec)",
				Math.toDegrees(previousAngularVelocityRadPerSec));
	}
	@Override
	public void reset() {
		previousAngularVelocityRadPerSec = 0.0;
		currentGyroDriftAmount = new Rotation2d();
		gyroSimulation.reset();
	}
	public static class GyroPhysicsSimulationResults {
		public double robotAngularVelocityRadPerSec, gForce;
		public boolean hasReading;
		public final Rotation2d[] odometryYawPositions = new Rotation2d[RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD];

		public GyroPhysicsSimulationResults() {
			robotAngularVelocityRadPerSec = 0.0;
			gForce = 0.0;
			hasReading = false;
			Arrays.fill(odometryYawPositions, new Rotation2d());
		}
	}
}
