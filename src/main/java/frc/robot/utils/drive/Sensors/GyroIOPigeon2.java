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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.selfCheck.drive.SelfCheckingPigeon2;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
	private static final int id = 0;
	private final Pigeon2 pigeon;
	private final StatusSignal<Double> yaw;
	private final StatusSignal<Double> accelX;
	private final StatusSignal<Double> accelY;
	private final Queue<Double> yawPositionQueue;
	private final StatusSignal<Double> yawVelocity;
	private double last_world_linear_accel_x, last_world_linear_accel_y;

	public GyroIOPigeon2() {
		pigeon = new Pigeon2(id, "rio");
		yaw = pigeon.getYaw();
		yawVelocity = pigeon.getAngularVelocityZWorld();
		accelX = pigeon.getAccelerationX();
		accelY = pigeon.getAccelerationY();
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(250);
		yawVelocity.setUpdateFrequency(100.0);
		accelX.setUpdateFrequency(100.0);
		accelY.setUpdateFrequency(100.0);
		pigeon.optimizeBusUtilization();
		yawPositionQueue = OdometryThread.registerSignalInput(yaw);
	}

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = BaseStatusSignal
				.refreshAll(yaw, yawVelocity, accelX, accelY).isOK();
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units
				.degreesToRadians(yawVelocity.getValueAsDouble());
		inputs.odometryYawPositions = yawPositionQueue.stream()
				.map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
		yawPositionQueue.clear();
		double curr_world_linear_accel_x = accelX.getValueAsDouble();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = accelY.getValueAsDouble();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			inputs.collisionDetected = true;
		} else {
			inputs.collisionDetected = false;
		}
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingPigeon2("IMU", pigeon));
		return hardware;
	}
}