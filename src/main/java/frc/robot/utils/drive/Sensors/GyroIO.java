// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
	@AutoLog
	public static class GyroIOInputs {
		public boolean connected = false;
		public Rotation2d yawPosition = new Rotation2d();
		public double[] odometryYawTimestamps = new double[] {};
		public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
		public double yawVelocityRadPerSec = 0.0;
		public boolean collisionDetected;
		public double gForceX = 0.0;
		public double gForceY = 0.0;

	}

	public default void updateInputs(GyroIOInputs inputs) {}

	public default void reset() {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
