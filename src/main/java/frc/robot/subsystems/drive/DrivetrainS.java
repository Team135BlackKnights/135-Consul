package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SubsystemChecker.SystemStatus;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.Position;
import java.util.List;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.HashMap;

public interface DrivetrainS extends Subsystem {
	/**
	 * Make the chassis go at a set speed
	 * 
	 * @param speeds the speed to set it to
	 */
	public static Field2d robotField = new Field2d();

	void setChassisSpeeds(ChassisSpeeds speeds);

	/**
	 * @return the ChassisSpeeds of the drivetrain
	 */
	ChassisSpeeds getChassisSpeeds();

	default void changeDeadband(double newDeadband) {
		DriveConstants.TrainConstants.kDeadband = newDeadband;
	}

	/**
	 * Reset the drivetrain's odometry to a particular pose
	 * 
	 * @param pose the pose to set it to
	 */
	void resetPose(Pose2d pose);

	/**
	 * Apply a particular vision measurement to the drivetrain
	 * 
	 * @param pose       the pose returned by the vision estimate
	 * @param timestamp  the timestamp of the pose
	 * @param estStdDevs the estimated std dev (pose's difference from the mean
	 *                      in x, y, and theta)
	 */
	void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs);

	/**
	 * @return the pose of the robot
	 */
	Pose2d getPose();

	/**
	 * Stops the drivetrain
	 */
	void stopModules();

	/**
	 * @return the angle of the drivetrain as a rotation2d (in degrees)
	 */
	Rotation2d getRotation2d();

	/** Returns the current yaw velocity (Z rotation) in radians per second. */
	public default double getYawVelocity() {
		return 0;
	}

	/**
	 * Returns the measured X, Y, and theta field velocities in meters per sec.
	 * The components of the twist are velocities and NOT changes in position.
	 */
	public Twist2d getFieldVelocity();

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kforward the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdDynamicDrive(Direction kforward);

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kforward the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdQuasistaticDrive(Direction kreverse);

	/**
	 * Reset the heading of the drivetrain
	 */
	void zeroHeading();

	/**
	 * @return if the gyro drivetrain is connected
	 */
	boolean isConnected();

	/**
	 * Checks gyros/accelerometers to check if a sudden movement has occured.
	 * 
	 * @return if a collision is detected
	 */
	boolean isCollisionDetected();

	HashMap<String, Double> getTemps();

	default Command getRunnableSystemCheckCommand() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getRunnableSystemCheckCommand'");
	}

	default SystemStatus getTrueSystemStatus() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getTrueSystemStatus'");
	}

	default List<ParentDevice> getDriveOrchestraDevices() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getDriveOrchestraDevices'");
	}

	/**
	 * Create a position wrapper which contains the positions, and the
	 * timestamps.
	 * 
	 * @param <T>       The type of position, MechanumWheelPositions or
	 *                     SwerveModulePositions[] or tank's.
	 * @param positions with both a timestamp and position.
	 * @return
	 */
	default <T> Position<T> getPositionsWithTimestamp(T positions) {
		double timestamp = Logger.getTimestamp();
		return new Position<>(positions, timestamp);
	}

	default double getCurrent() { return 0; }

	default boolean[] isSkidding() {
		return new boolean[] { false, false, false, false
		};
	}

	@Override
	default void periodic() {
		robotField.setRobotPose(getPose());
		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
			robotField.getObject("target pose").setPose(pose);
		});
		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			Logger.recordOutput("Odometry/Trajectory",
					poses.toArray(new Pose2d[poses.size()]));
			robotField.getObject("path").setPoses(poses);
		});
		SmartDashboard.putData(robotField);
	}
}
