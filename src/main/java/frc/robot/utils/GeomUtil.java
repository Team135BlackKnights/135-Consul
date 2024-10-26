package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class GeomUtil {
	/**
	 * Creates a pure translating transform
	 *
	 * @param x The x component of the translation
	 * @param y The y component of the translation
	 * @return The resulting transform
	 */
	public static Transform2d translationToTransform(double x, double y) {
		return new Transform2d(new Translation2d(x, y), new Rotation2d());
	}

	public enum ApproachDirection {
		FRONT, RIGHT, BACK, LEFT
	}

	/**
	 * Works similar to swerveS.optimize but for any angle
	 * 
	 * @param angle the angle to check
	 * @return the optimized distance to rotate to reach an ideal angle
	 */
	public static double closerAngleToZero(double angle) {
		// Normalize the angle to be within the range of -180 to 180 degrees
		double normalizedAngle = angle % 360;
		if (normalizedAngle > 180) {
			normalizedAngle -= 360;
		} else if (normalizedAngle < -180) {
			normalizedAngle += 360;
		}
		// Return the closer angle to zero
		return (Math.abs(normalizedAngle) <= 180) ? normalizedAngle
				: -normalizedAngle;
	}

	/**
	 * Modified PID Controller, but for our DriveToAITarget
	 * 
	 * @param x the x distance from target
	 * @return the x speed
	 */
	public static double speedMapper(double x) {
		// Define the parameters for the sigmoid function
		double x0 = 20; // Inches where the function starts to rise significantly
		double k = 0.1; // Steepness of the curve
		// Apply the sigmoid function to map x to the range [0, 1]
		double y = 1 / (1 + Math.exp(-k * (x - x0)));
		// Adjust the output to meet your specific points
		if (x >= 40) {
			y = 1;
		}
		return y;
	}

	public static double distancePose(Pose2d current, Pose2d other) {
		return other.getTranslation().minus(current.getTranslation()).getNorm();
	}

	public static Rotation2d rotationFromCurrentToTarget(Translation2d currentPose,
	Translation2d targetPose, ApproachDirection direction) {
		// Extract positions
		double dx = targetPose.getX() - currentPose.getX();
		double dy = targetPose.getY() - currentPose.getY();
		// Compute angle from currentPose to targetPose
		double angle = Math.atan2(dy, dx);
		// Convert angle from radians to degrees
		switch (direction) {
		case RIGHT:
			angle -= Math.PI / 2;
			break;
		case BACK:
			angle += Math.PI;
			break;
		case LEFT:
			angle += Math.PI / 2;
			break;
		case FRONT:
		default:
			// No adjustment needed for FRONT
			break;
		}
		double angleDegrees = Math.toDegrees(angle);
		// Create a Rotation2d object from the angle
		Rotation2d rotationFromCurrentToTarget = new Rotation2d(
				Units.degreesToRadians(angleDegrees));
		return rotationFromCurrentToTarget;
	}

	/**
	 * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and
	 * Z). chain
	 *
	 * @param speeds The original translation
	 * @return The resulting translation
	 */
	public static Twist2d toTwist2d(ChassisSpeeds speeds) {
		return new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond);
	}

	/**
	 * Calculates the robot- relative Pose2d based on Limelight readings.**
	 * 
	 * @param tx              The horizontal angle offset to the target in
	 *                           degrees.*
	 * @param ty              The vertical angle offset to the target in
	 *                           degrees.*
	 * @param limelightHeight The height of the Limelight from the floor in
	 *                           meters.*
	 * @param targetHeight    The height of the target from the floor in meters.*
	 * @param limelightAngle  The angle of the Limelight relative to the floor in
	 *                           degrees.*@return The robot-relative Pose2d.
	 */
	public static Pose2d calculateRobotRelativePose2d(double tx, double ty,
			double limelightHeight, double targetHeight, double limelightAngle) {
		// Convert degrees to radians
		double txRad = Math.toRadians(tx);
		double tyRad = Math.toRadians(ty);
		double limelightAngleRad = Math.toRadians(limelightAngle);
		// Calculate distance to the target
		double distance = (targetHeight - limelightHeight)
				/ Math.tan(limelightAngleRad + tyRad);
		// Calculate x and y distances
		double x = distance * Math.cos(txRad);
		double y = distance * Math.sin(txRad);
		// Create and return the Pose2d (robot-relative)
		return new Pose2d(x, y, new Rotation2d(txRad));
	}

	/**
	 * Calculates the field-relative Pose2d based on the robot's current pose and
	 * Limelight readings.
	 *
	 * @param robotPose       The current field-relative pose of the robot.
	 * @param tx              The horizontal angle offset to the target in
	 *                           degrees.
	 * @param ty              The vertical angle offset to the target in degrees.
	 * @param limelightHeight The height of the Limelight from the floor in
	 *                           meters.
	 * @param targetHeight    The height of the target from the floor in meters.
	 * @param limelightAngle  The angle of the Limelight relative to the floor in
	 *                           degrees.
	 * @return The field-relative Pose2d.
	 */
	public static Pose2d calculateFieldRelativePose2d(Pose2d robotPose,
			double tx, double ty, double limelightHeight, double targetHeight,
			double limelightAngle) {
		Pose2d robotRelativePose = calculateRobotRelativePose2d(tx, ty,
				limelightHeight, targetHeight, limelightAngle);
		double x_r = robotPose.getX();
		double y_r = robotPose.getY();
		double theta_r = robotPose.getRotation().getRadians();
		double x = robotRelativePose.getX();
		double y = robotRelativePose.getY();
		// Calculate field-relative coordinates
		double x_f = x_r + x * Math.cos(theta_r) - y * Math.sin(theta_r);
		double y_f = y_r + x * Math.sin(theta_r) + y * Math.cos(theta_r);
		double theta_f = theta_r + robotRelativePose.getRotation().getRadians();
		// Create and return the field-relative Pose2d
		return new Pose2d(x_f, y_f, new Rotation2d(theta_f));
	}

	/**
	 * Calculates the field-relative Pose3d based on the robot's current pose and
	 * Limelight readings.
	 *
	 * @param robotPose       The current field-relative pose of the robot.
	 * @param tx              The horizontal angle offset to the target in
	 *                           degrees.
	 * @param ty              The vertical angle offset to the target in degrees.
	 * @param limelightHeight The height of the Limelight from the floor in
	 *                           meters.
	 * @param targetHeight    The height of the target from the floor in meters.
	 * @param limelightAngle  The angle of the Limelight relative to the floor in
	 *                           degrees.
	 * @return The field-relative Pose3d.
	 */
	public static Pose3d calculateFieldRelativePose3d(Pose2d robotPose,
			double tx, double ty, double limelightHeight, double targetHeight,
			double limelightAngle) {
		Pose2d robotRelativePose2d = calculateRobotRelativePose2d(tx, ty,
				limelightHeight, targetHeight, limelightAngle);
		// Extract the robot's current field-relative translation and rotation
		Translation3d robotTranslation = new Translation3d(robotPose.getX(),
				robotPose.getY(), 0);
		// Extract the robot-relative translation
		Translation3d relativeTranslation = new Translation3d(
				robotRelativePose2d.getX(), robotRelativePose2d.getY(),
				targetHeight);
		// Rotate the relative translation to the field coordinate system
		double x_f = robotTranslation.getX()
				+ relativeTranslation.getX()
						* Math.cos(robotPose.getRotation().getRadians())
				- relativeTranslation.getY()
						* Math.sin(robotPose.getRotation().getRadians());
		double y_f = robotTranslation.getY()
				+ relativeTranslation.getX()
						* Math.sin(robotPose.getRotation().getRadians())
				+ relativeTranslation.getY()
						* Math.cos(robotPose.getRotation().getRadians());
		double z_f = relativeTranslation.getZ();
		// Calculate the rotation for the field-relative pose (not influenced by robot rotation)
		Rotation3d fieldRotation = new Rotation3d(0, 0, Math.toRadians(tx));
		// Create and return the field-relative Pose3d
		return new Pose3d(new Translation3d(x_f, y_f, z_f), fieldRotation);
	}

	/**
	 * @param currentPose the robot pose
	 * @param objectPose  the object, as a pose3d
	 * @return distance in meters
	 */
	public static double calculateDistanceFromPose3d(Pose2d currentPose,
			Pose3d objectPose) {
		return currentPose.getTranslation()
				.getDistance(objectPose.getTranslation().toTranslation2d());
	}

	/**
	 * @param currentTranslation the robot translation
	 * @param objectTranslation  the object, as a translation2d
	 * @return distance in meters
	 */
	public static double calculateDistanceFromTranslation2d(
			Translation2d currentTranslation, Translation2d objectTranslation) {
		return currentTranslation.getDistance(objectTranslation);
	}
}
