// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.utils.drive.Sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.utils.selfCheck.drive.SelfCheckingPigeon2;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
	// Filter to smooth accelerometer data. Set to half the Odometry frequency in
	// order to still remain responsive (pigeon updates at n updates/sec, so we want
	// to average 2 updates/sec)
	private final LinearFilter hangLimiter = LinearFilter.movingAverage((int) DriveConstants.TrainConstants.odomHz / 2);
	private final Pigeon2 pigeon;
	private final StatusSignal<Angle> yaw;
	private final StatusSignal<LinearAcceleration> accelX;
	private final StatusSignal<LinearAcceleration> accelY;
	private final StatusSignal<LinearAcceleration> accelZ;
	private final Queue<Double> yawPositionQueue;
	private final StatusSignal<AngularVelocity> yawVelocity;
	//Zpos in meters. I could go through and write the IO for this, 
	//but we're literally using this to play miley cyrus on hang so I'm not going to bother -N
	private double zPosition = 0.0;
	private double last_world_linear_accel_x, last_world_linear_accel_y;

	public GyroIOPigeon2() {
		if (DriveConstants.driveCanBus == null) {
			pigeon = new Pigeon2(DriveConstants.kGyroPort);
		} else {
			pigeon = new Pigeon2(DriveConstants.kGyroPort, DriveConstants.driveCanBus);
		}
		yaw = pigeon.getYaw();
		yawVelocity = pigeon.getAngularVelocityZWorld();
		accelX = pigeon.getAccelerationX();
		accelY = pigeon.getAccelerationY();
		accelZ = pigeon.getAccelerationZ();
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(DriveConstants.TrainConstants.odomHz);
		yawVelocity.setUpdateFrequency(50.0);
		accelX.setUpdateFrequency(50.0);
		accelY.setUpdateFrequency(50.0);
		pigeon.optimizeBusUtilization();
		yawPositionQueue = OdometryThread.registerSignalInput(yaw);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = BaseStatusSignal
				.refreshAll(yaw, yawVelocity, accelX, accelY, accelZ).isOK();
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble()).plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		inputs.yawVelocityRadPerSec = Units
				.degreesToRadians(yawVelocity.getValueAsDouble());
		inputs.odometryYawPositions = yawPositionQueue.stream()
		.map((Double value) -> Rotation2d.fromDegrees(value).plus(DriveConstants.TrainConstants.robotOffsetAngleDirection))
		.toArray(Rotation2d[]::new);
		yawPositionQueue.clear();
		double curr_world_linear_accel_x = accelX.getValueAsDouble();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = accelY.getValueAsDouble();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		inputs.gForceX = currentJerkX;
		inputs.gForceY = currentJerkY;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			inputs.collisionDetected = true;
		} else {
			inputs.collisionDetected = false;
		}
		zPosition += Math.pow(hangLimiter.calculate(accelZ.getValueAsDouble())*.5,2);
	}

	@Override
	public void reset() {
		pigeon.reset();
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingPigeon2("IMU", pigeon));
		return hardware;
	}

	public double getZPosition() {
		return zPosition;
	}
}