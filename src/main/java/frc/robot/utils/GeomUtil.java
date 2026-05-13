package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.Swerve.TxTyPoseRecord;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.maths.TimeUtil;
@SuppressWarnings("unused") // There are some unused methods/variables that are still useful to keep around for testing and tuning purposes
public class GeomUtil {
	private record BoundaryProximity(
			double distanceMeters,
			Translation2d directionTowardBoundary,
			Translation2d closestPoint,
			String boundaryName) {
	}

	public static double interp(double y0, double y1, double x, double x0, double x1) {
		if (x1 == x0)
			return y0;

		double t = (x - x0) / (x1 - x0);

		// Clamp t to [0, 1]
		if (t < 0.0)
			t = 0.0;
		else if (t > 1.0)
			t = 1.0;

		return y0 + (y1 - y0) * t;
	}

	/// Make sure the given speeds are ROBOT relative.
	public static ChassisSpeeds avoidRobots(ChassisSpeeds speeds) {
		final double MAX_AGE_SECONDS = 2.0;
		final double BASE_AVOID_MARGIN_M = 0.5; // previous constant
		final double MAX_EXTRA_MARGIN_M = 1.2; // additional margin at top approach (tunable)
		final double MAX_AVOID_SPEED = DriveConstants.kMaxSpeedMetersPerSecond * 10;

		Pose2d ourPose = RobotContainer.drivetrainS.getLookAheadPose();
		double now = TimeUtil.getLogTimeSeconds();

		double avoidRobotX = 0.0;
		double avoidRobotY = 0.0;
		boolean anyActive = false;

		double len = DriveConstants.kBumperToBumperLength;
		double wid = DriveConstants.kBumperToBumperWidth;

		double thetaLoop = RobotContainer.drivetrainS.getRotation2d().getRadians();
		double cosLoop = Math.cos(-thetaLoop);
		double sinLoop = Math.sin(-thetaLoop);

		double maxDecel = DriveConstants.maxTranslationalAcceleration.get();

		// measured & commanded translational speed magnitude (global)
		ChassisSpeeds measured = RobotContainer.drivetrainS.getChassisSpeeds();
		double measuredSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
		double commandedSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
		double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

		// Log the globals at least
		Logger.recordOutput("Controller/Avoidance/MeasuredSpeed", measuredSpeed);
		Logger.recordOutput("Controller/Avoidance/CommandedSpeed", commandedSpeed);

		/*for (TxTyPoseRecord otherRobotPose : ((Swerve) RobotContainer.drivetrainS).getOpposingRobotPoses()) {
			Pose3d other3 = otherRobotPose.pose();
			if (other3 == null)
				continue;
			double z = other3.getTranslation().getZ();
			if (z > .5)
				continue;
			double age = now - otherRobotPose.timestamp();
			if (age > MAX_AGE_SECONDS)
				continue;

			double otherX = other3.getTranslation().getX();
			double otherY = other3.getTranslation().getY();

			double dx = ourPose.getX() - otherX;
			double dy = ourPose.getY() - otherY;
			double dist = Math.hypot(dx, dy);

			if (dist <= 1e-6) {
				dx = 1.0;
				dy = 0.0;
				dist = 1.0;
			}

			// field -> robot rotation unit vector for this target (unit from us->them)
			double uxField = (otherX - ourPose.getX()) / dist;
			double uyField = (otherY - ourPose.getY()) / dist;
			double uxRobot = cosLoop * uxField - sinLoop * uyField;
			double uyRobot = sinLoop * uxField + cosLoop * uyField;

			// compute approach-projection for commanded and measured velocities
			double cmdAlong = speeds.vxMetersPerSecond * uxRobot + speeds.vyMetersPerSecond * uyRobot;
			double measAlong = measured.vxMetersPerSecond * uxRobot + measured.vyMetersPerSecond * uyRobot;

			// Use the larger positive projection (if any) to decide "aiming"
			double approachAlong = Math.max(0.0, Math.max(cmdAlong, measAlong));

			// compute per-target dynamic margin: only expand if approachAlong > 0
			double margin;
			if (approachAlong <= 0.0) {
				margin = BASE_AVOID_MARGIN_M;
			} else {
				// scale extra margin by how big the approach is relative to max speed
				double approachScale = Math.min(1.0, approachAlong / Math.max(1e-6, maxSpeed));
				margin = BASE_AVOID_MARGIN_M + MAX_EXTRA_MARGIN_M * approachScale;
			}
			// clamp margin for safety
			margin = Math.max(BASE_AVOID_MARGIN_M, Math.min(BASE_AVOID_MARGIN_M + MAX_EXTRA_MARGIN_M, margin));

			double halfLen = (len * 0.5) + margin;
			double halfWid = (wid * 0.5) + margin;

			double absDx = Math.abs(dx);
			double absDy = Math.abs(dy);

			if (absDx < halfLen && absDy < halfWid) {
				double overlapX = Math.max(0.0, halfLen - absDx);
				double overlapY = Math.max(0.0, halfWid - absDy);

				double strengthX = Math.min(1.0, overlapX / halfLen);
				double strengthY = Math.min(1.0, overlapY / halfWid);

				double strength = Math.max(strengthX, strengthY);
				double mag = strength * MAX_AVOID_SPEED;

				// inward motion to consider (use max of cmd/meas)
				double inwardAlong = Math.max(0.0, Math.max(cmdAlong, measAlong));
				if (inwardAlong > 0.0) {
					double minOverlap = Math.min(overlapX, overlapY);

					// estimate stopping distance from current measured speed along approach
					double stoppingDist = (measAlong * measAlong) / (2.0 * Math.max(1e-3, maxDecel));

					double desiredAlong;
					if (stoppingDist > minOverlap) {
						// emergency braking (unchanged behavior)
						double brakeVel = Math.min(maxSpeed, Math.max(0.5 * maxSpeed, measAlong));
						desiredAlong = -brakeVel;
					} else {
						double cancel = Math.min(mag, inwardAlong);
						desiredAlong = cmdAlong - cancel;
					}

					double reduction = Math.max(0.0, cmdAlong - desiredAlong);
					reduction = Math.min(reduction, mag);

					avoidRobotX += -uxRobot * reduction;
					avoidRobotY += -uyRobot * reduction;
					anyActive = true;
					Logger.recordOutput("Controller/Avoidance/OtherAppliedReduction", reduction);
				}
				Logger.recordOutput("Controller/Avoidance/OtherAge", age);
			}
		} // end loop*/

		if (!anyActive) {
			return speeds;
		}

		double newVx = speeds.vxMetersPerSecond + avoidRobotX;
		double newVy = speeds.vyMetersPerSecond + avoidRobotY;

		double maxSpeedClamp = DriveConstants.kMaxSpeedMetersPerSecond;
		if (Math.abs(newVx) > maxSpeedClamp)
			newVx = Math.signum(newVx) * maxSpeedClamp;
		if (Math.abs(newVy) > maxSpeedClamp)
			newVy = Math.signum(newVy) * maxSpeedClamp;

		double newOmega = speeds.omegaRadiansPerSecond;

		ChassisSpeeds out = new ChassisSpeeds(newVx, newVy, newOmega);
		Logger.recordOutput("Controller/Avoidance/AppliedVX", avoidRobotX);
		Logger.recordOutput("Controller/Avoidance/AppliedVY", avoidRobotY);
		Logger.recordOutput("Controller/Avoidance/ResultVX", newVx);
		Logger.recordOutput("Controller/Avoidance/ResultVY", newVy);

		return out;
	}

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

	public static Transform2d poseToTransform(Pose2d pose) {
		return new Transform2d(pose.getX(), pose.getY(), pose.getRotation());
	}

	public static Transform3d poseToTransform3d(Pose3d pose) {
		return new Transform3d(pose.getX(), pose.getY(), pose.getZ(), pose.getRotation());
	}

	/**
	 * Creates a pure translating transform
	 */
	public static Transform3d poseToTransform(Pose3d pose) {
		return new Transform3d(pose.getX(), pose.getY(), pose.getZ(), pose.getRotation());
	}

	public static Translation2d projectObjectObservationToField(
			Pose2d robotPose,
			Pose3d robotToCamera,
			Rotation2d tx,
			Rotation2d ty,
			double distanceMeters) {
		double clampedDistanceMeters = Math.max(0.0, distanceMeters);
		Translation3d cameraToTarget = new Pose3d(
				Translation3d.kZero,
				new Rotation3d(0.0, ty.getRadians(), -tx.getRadians()))
				.transformBy(new Transform3d(new Translation3d(clampedDistanceMeters, 0.0, 0.0), Rotation3d.kZero))
				.getTranslation();
		Pose3d fieldCameraPose = new Pose3d(robotPose).transformBy(poseToTransform(robotToCamera));
		Translation3d fieldTarget = fieldCameraPose.getTranslation()
				.plus(cameraToTarget.rotateBy(fieldCameraPose.getRotation()));
		return fieldTarget.toTranslation2d();
	}

	/**
	 * Creates a pure translating pose3d
	 */
	public static Pose3d transformToPose(Transform3d transform) {
		return new Pose3d(transform.getX(), transform.getY(), transform.getZ(), transform.getRotation());
	}

	public enum ApproachDirection {
		FRONT(0),
		FRONT_RIGHT(Math.PI / 4),
		RIGHT(Math.PI / 2),
		BACK_RIGHT(3 * Math.PI / 4),
		BACK(Math.PI),
		BACK_LEFT(-3 * Math.PI / 4),
		LEFT(-Math.PI / 2),
		FRONT_LEFT(-Math.PI / 4);

		private final double angle;

		// Constructor to initialize the angle
		ApproachDirection(double angle) {
			this.angle = angle;
		}

		// Getter method to retrieve the angle
		public double getAngle() {
			return angle;
		}
	}

	/**
	 * Works similar to swerveS.optimize but for any angle
	 * 
	 * @param angle the angle to check
	 * @return the optimized distance to rotate to reach an ideal angle IN DEGREES
	 */
	public static double closerAngleToZero(Rotation2d angle) {
		// Normalize the angle to be within the range of -180 to 180 degrees
		double angleDegrees = angle.getDegrees();
		double normalizedAngle = MathUtil.inputModulus(angleDegrees, -180, 180);
		return normalizedAngle;
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
		angle += direction.getAngle();
		// wrap the angle to be within -pi to pi
		Rotation2d rotationFromCurrentToTarget = new Rotation2d(MathUtil.inputModulus(angle, -Math.PI, Math.PI));
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

	public static double distanceToNearestFieldEdge(Translation2d point) {
		double left = point.getX();
		double right = FieldConstants.FIELD_WIDTH - point.getX();
		double bottom = point.getY();
		double top = FieldConstants.FIELD_HEIGHT - point.getY();
		return Math.min(Math.min(left, right), Math.min(bottom, top));
	}

	public static Translation2d directionTowardNearestFieldEdge(Translation2d point) {
		double left = point.getX();
		double right = FieldConstants.FIELD_WIDTH - point.getX();
		double bottom = point.getY();
		double top = FieldConstants.FIELD_HEIGHT - point.getY();

		double nearest = left;
		Translation2d direction = new Translation2d(-1.0, 0.0);

		if (right < nearest) {
			nearest = right;
			direction = new Translation2d(1.0, 0.0);
		}
		if (bottom < nearest) {
			nearest = bottom;
			direction = new Translation2d(0.0, -1.0);
		}
		if (top < nearest) {
			direction = new Translation2d(0.0, 1.0);
		}

		return direction;
	}

	private static BoundaryProximity getFieldEdgeProximity(Translation2d point) {
		double left = point.getX();
		double right = FieldConstants.FIELD_WIDTH - point.getX();
		double bottom = point.getY();
		double top = FieldConstants.FIELD_HEIGHT - point.getY();

		BoundaryProximity best = new BoundaryProximity(
				left,
				new Translation2d(-1.0, 0.0),
				new Translation2d(0.0, point.getY()),
				"FieldLeft");

		if (right < best.distanceMeters()) {
			best = new BoundaryProximity(
					right,
					new Translation2d(1.0, 0.0),
					new Translation2d(FieldConstants.FIELD_WIDTH, point.getY()),
					"FieldRight");
		}
		if (bottom < best.distanceMeters()) {
			best = new BoundaryProximity(
					bottom,
					new Translation2d(0.0, -1.0),
					new Translation2d(point.getX(), 0.0),
					"FieldBottom");
		}
		if (top < best.distanceMeters()) {
			best = new BoundaryProximity(
					top,
					new Translation2d(0.0, 1.0),
					new Translation2d(point.getX(), FieldConstants.FIELD_HEIGHT),
					"FieldTop");
		}
		return best;
	}

	private static BoundaryProximity getAxisAlignedRectangleProximity(
			Translation2d point,
			Translation2d cornerA,
			Translation2d cornerB,
			String boundaryName) {
		double minX = Math.min(cornerA.getX(), cornerB.getX());
		double maxX = Math.max(cornerA.getX(), cornerB.getX());
		double minY = Math.min(cornerA.getY(), cornerB.getY());
		double maxY = Math.max(cornerA.getY(), cornerB.getY());

		double clampedX = MathUtil.clamp(point.getX(), minX, maxX);
		double clampedY = MathUtil.clamp(point.getY(), minY, maxY);
		Translation2d closestPoint = new Translation2d(clampedX, clampedY);
		Translation2d toBoundary = closestPoint.minus(point);
		double distanceMeters = toBoundary.getNorm();

		if (distanceMeters > 1e-9) {
			return new BoundaryProximity(
					distanceMeters,
					toBoundary.div(distanceMeters),
					closestPoint,
					boundaryName);
		}

		double distanceToLeft = Math.abs(point.getX() - minX);
		double distanceToRight = Math.abs(maxX - point.getX());
		double distanceToBottom = Math.abs(point.getY() - minY);
		double distanceToTop = Math.abs(maxY - point.getY());

		BoundaryProximity best = new BoundaryProximity(
				distanceToLeft,
				new Translation2d(-1.0, 0.0),
				new Translation2d(minX, point.getY()),
				boundaryName + "/LeftFace");
		if (distanceToRight < best.distanceMeters()) {
			best = new BoundaryProximity(
					distanceToRight,
					new Translation2d(1.0, 0.0),
					new Translation2d(maxX, point.getY()),
					boundaryName + "/RightFace");
		}
		if (distanceToBottom < best.distanceMeters()) {
			best = new BoundaryProximity(
					distanceToBottom,
					new Translation2d(0.0, -1.0),
					new Translation2d(point.getX(), minY),
					boundaryName + "/BottomFace");
		}
		if (distanceToTop < best.distanceMeters()) {
			best = new BoundaryProximity(
					distanceToTop,
					new Translation2d(0.0, 1.0),
					new Translation2d(point.getX(), maxY),
					boundaryName + "/TopFace");
		}
		return best;
	}

	private static BoundaryProximity getNearestDriveBoundary(Translation2d point) {
		BoundaryProximity best = getFieldEdgeProximity(point);

		BoundaryProximity allianceHub = getAxisAlignedRectangleProximity(
				point,
				FieldConstants.Hub.nearRightCorner,
				FieldConstants.Hub.farLeftCorner,
				"AllianceHub");
		if (allianceHub.distanceMeters() < best.distanceMeters()) {
			best = allianceHub;
		}

		BoundaryProximity oppositeHub = getAxisAlignedRectangleProximity(
				point,
				FieldConstants.Hub.oppNearRightCorner,
				FieldConstants.Hub.oppFarLeftCorner,
				"OppositeHub");
		if (oppositeHub.distanceMeters() < best.distanceMeters()) {
			best = oppositeHub;
		}

		return best;
	}

	public static Translation2d limitVelocityTowardFieldEdge(
			Translation2d desiredFieldVelocity,
			Translation2d robotPosition,
			Translation2d targetPosition,
			double slowDistanceMeters,
			double maxTowardWallSpeedMetersPerSec) {
		if (desiredFieldVelocity.getNorm() <= 1e-9 || slowDistanceMeters <= 0.0) {
			return desiredFieldVelocity;
		}

		Translation2d referencePoint = robotPosition;
		BoundaryProximity nearestBoundary = getNearestDriveBoundary(robotPosition);
		if (targetPosition != null) {
			BoundaryProximity targetBoundary = getNearestDriveBoundary(targetPosition);
			if (targetBoundary.distanceMeters() < nearestBoundary.distanceMeters()) {
				referencePoint = targetPosition;
				nearestBoundary = targetBoundary;
			}
		}

		double referenceDistance = nearestBoundary.distanceMeters();
		if (referenceDistance >= slowDistanceMeters) {
			return desiredFieldVelocity;
		}

		Translation2d towardWallDirection = nearestBoundary.directionTowardBoundary();
		double towardWallSpeed =
				desiredFieldVelocity.getX() * towardWallDirection.getX()
						+ desiredFieldVelocity.getY() * towardWallDirection.getY();
		if (towardWallSpeed <= 0.0) {
			return desiredFieldVelocity;
		}

		double allowedTowardWallSpeed =
				interp(
							maxTowardWallSpeedMetersPerSec,
							desiredFieldVelocity.getNorm(),
							referenceDistance,
							0.0,
							slowDistanceMeters);
		double limitedTowardWallSpeed = Math.min(towardWallSpeed, allowedTowardWallSpeed);
		Logger.recordOutput("Drive/BoundaryLimiter/ReferencePoint", new Pose2d(referencePoint, new Rotation2d()));
		Logger.recordOutput("Drive/BoundaryLimiter/ClosestBoundaryPoint",
				new Pose2d(nearestBoundary.closestPoint(), new Rotation2d()));
		Logger.recordOutput("Drive/BoundaryLimiter/DistanceMeters", referenceDistance);
		Logger.recordOutput("Drive/BoundaryLimiter/BoundaryName", nearestBoundary.boundaryName());
		Translation2d tangentialVelocity =
				desiredFieldVelocity.minus(towardWallDirection.times(towardWallSpeed));
		return tangentialVelocity.plus(towardWallDirection.times(limitedTowardWallSpeed));
	}

	public static double applyX(double x) {
		return shouldFlip() ? FieldConstants.FIELD_WIDTH - x : x;
	}

	public static double applyX(double x, boolean forceFlip) {
		return shouldFlip() || forceFlip ? FieldConstants.FIELD_WIDTH - x : x;
	}

	public static Transform2d toTransform2d(Translation2d translation) {
		return new Transform2d(translation, new Rotation2d());
	}

	public static Transform2d toTransform2d(double x, double y) {
		return new Transform2d(x, y, new Rotation2d());
	}

	public static double applyY(double y) {
		return shouldFlip() ? FieldConstants.FIELD_HEIGHT - y : y;
	}

	public static double applyY(double y, boolean forceFlip) {
		return shouldFlip() || forceFlip ? FieldConstants.FIELD_HEIGHT - y : y;
	}

	public static Translation2d apply(Translation2d translation) {
		return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
	}

	public static Rotation2d apply(Rotation2d rotation) {
		return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
	}

	public static Pose2d apply(Pose2d pose, boolean forceFlip) {
		if (pose == null) {
			return new Pose2d(); // default to left
		}
		return shouldFlip() || forceFlip
				? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
				: pose;
	}

	public static Pose3d flip(Pose3d toFlip) {
		return new Pose3d(
				new Translation3d(FieldConstants.FIELD_WIDTH - toFlip.getX(),
						FieldConstants.FIELD_HEIGHT - toFlip.getY(), toFlip.getZ()),
				toFlip.getRotation());
	}

	public static Translation3d apply(Translation3d translation, boolean forceFlip) {
		return new Translation3d(applyX(translation.getX(), forceFlip), applyY(translation.getY(), forceFlip),
				translation.getZ());
	}

	public static Translation2d apply(Translation2d translation, boolean forceFlip) {
		return new Translation2d(applyX(translation.getX(), forceFlip), applyY(translation.getY(), forceFlip));
	}

	public static boolean shouldFlip() {
		return DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
	}
}
