package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.vision.SelfCheckingLimelight;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;

public class Vision extends SubsystemChecker {
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private boolean staleReading = false;
	private Pose2d lastOdomPose = new Pose2d(0, 0, new Rotation2d(0));

	public Vision(VisionIO... io) {
		this.io = io;
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < io.length; i++) {
			this.inputs[i] = new VisionIOInputsAutoLogged();
		}
		registerSelfCheckHardware();
	}

	@Override
	public void periodic() {
		long timestamp = System.currentTimeMillis();
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}
		Logger.recordOutput("SystemStatus/Periodic/VisionInputsMS", System.currentTimeMillis() - timestamp);
		// Initialize logging values
		timestamp = System.currentTimeMillis();
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();
		Pose2d currentOdomPose = RobotContainer.drivetrainS.getPose();
		staleReading = (Math.abs(currentOdomPose.getX() - lastOdomPose.getX()) < VisionConstants.maxStaleReadingXMeters
				|| Math.abs(currentOdomPose.getY() - lastOdomPose.getY()) < VisionConstants.maxStaleReadingYMeters
				|| Math.abs(currentOdomPose.getRotation().getDegrees()
						- lastOdomPose.getRotation().getDegrees()) < VisionConstants.maxStaleReadingRotation);
		Logger.recordOutput("Vision/Stale", staleReading);
		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			if (!inputs[cameraIndex].connected) {
				addFault("NO heartbeat detected from " + inputs[cameraIndex].name);
			}

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();
			double averageTrust = 0.0;
			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = VisionConstants.kTagLayout.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
				averageTrust += VisionConstants.FieldConstants.aprilTagOffsets[tagId]; // default is 1.0
			}
			averageTrust /= inputs[cameraIndex].tagIds.length;
			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
						|| (observation.tagCount() == 1
								&& observation.ambiguity() > VisionConstants.maxAmbiguity) // Cannot be high ambiguity
						|| Math.abs(observation.pose().getZ()) > VisionConstants.maxZError // Must have realistic Z
																							// coordinate
						|| Math.abs(observation.pose().getRotation().toRotation2d().getDegrees()
								- RobotContainer.drivetrainS.getPose().getRotation()
										.getDegrees()) > VisionConstants.maxYawError // Must have realistic yaw
						// Must be reasonable apriltag trust
						|| averageTrust < VisionConstants.FieldConstants.kFieldTagMinTrust

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > VisionConstants.kTagLayout.getFieldWidth();

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					if (!staleReading) {
						// update tag trusts
						for (int tag : inputs[cameraIndex].tagIds) {
							VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math.min(10, // max of 10x deviation
									VisionConstants.FieldConstants.aprilTagOffsets[tag]
											+ .002);
						}
					}
					continue;
				}

				// Calculate standard deviations
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 1.0) / observation.tagCount();
				double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
				double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
				if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
					linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
					angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
				}
				linearStdDev *= averageTrust;
				angularStdDev *= averageTrust;
				// Send vision observation
				addVisionMeasurement(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
				if (!staleReading) {
					// update tag trusts
					for (int tag : inputs[cameraIndex].tagIds) {
						VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math.max(1,
								VisionConstants.FieldConstants.aprilTagOffsets[tag]
										- .002);
					}
				}
			}

			// Log camera datadata
			Logger.recordOutput(
					"Vision/" + inputs[cameraIndex].name + "/TagPoses",
					tagPoses.toArray(new Pose3d[tagPoses.size()]));
			Logger.recordOutput(
					"Vision/" + inputs[cameraIndex].name + "/RobotPoses",
					robotPoses.toArray(new Pose3d[robotPoses.size()]));
			Logger.recordOutput(
					"Vision/" + inputs[cameraIndex].name + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
			Logger.recordOutput(
					"Vision/" + inputs[cameraIndex].name + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput(
				"Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
		Logger.recordOutput("Vision/FieldTrusts", VisionConstants.FieldConstants.aprilTagOffsets);
		Logger.recordOutput("SystemStatus/Periodic/VisionProcessMS", System.currentTimeMillis() - timestamp);
	}

	/**
	 * Adds the vision measurement to the poseEstimator. This is the same as the
	 * default poseEstimator function, we just do it this way to fit our block
	 * template structure
	 */
	public void addVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		RobotContainer.drivetrainS.newVisionMeasurement(pose, timestamp,
				estStdDevs);
	}

	public boolean objectVisionOkay() {
		return LimelightHelpers.getLatestResults(VisionConstants.limelightName).error == "";
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(new ArrayList<SelfChecking>(
				List.of(new SelfCheckingLimelight(VisionConstants.limelightName))));
	}

	/**
	 * Calculate the distance from the photon vision camera to the target
	 * 
	 * @param cam the camera index, as defined in RobotContainer.java
	 */
	public double calculateDistanceFromCam(int cam) {
		if (inputs[cam].targetObservations.length == 0) {
			return 0;
		}
		double ty = inputs[cam].targetObservations[0].ty().getDegrees();
		return calculateDistanceFromtY(ty);
	}

	/**
	 * Get the latest target observation from the photon vision camera
	 * 
	 * @param cam the camera index, as defined in RobotContainer.java
	 */
	public TargetObservation getLatestTargetObservation(CameraID cam) {
		if (inputs[cam.ordinal()].targetObservations.length == 0) {
			return new TargetObservation(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 0,
					new Transform3d(), 0.0);
		}
		return new TargetObservation(inputs[cam.ordinal()].targetObservations[0].tx(),
				inputs[cam.ordinal()].targetObservations[0].ty(), inputs[cam.ordinal()].targetObservations[0].id(), inputs[cam.ordinal()].targetObservations[0].cameraToTarget(), inputs[cam.ordinal()].targetObservations[0].timestamp());
	}

	/**
	 * Computes the distance in inches from the limelight network table entry
	 * 
	 * @param tY the tY measurement from the limelight network table entry (pitch
	 *           degrees)
	 * @return distance in inches
	 */
	public static double calculateDistanceFromtY(double tY) {
		// Calculate the angle using trigonometry
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = VisionConstants.limeLightAngleOffsetDegrees;
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = VisionConstants.limelightLensHeightoffFloorInches;
		// distance from the target to the floor
		double goalHeightInches = 0; // if multiple targets, make this an argument
		double angleToGoalDegrees = limelightMountAngleDegrees + tY;
		// calculate distance
		double distance = (goalHeightInches - limelightLensHeightInches)
				/ Math.tan(Units.degreesToRadians(angleToGoalDegrees));
		return Math.abs(distance); // incase somehow the angle became pos when the object is below the target, and
									// vice versa.
	}

	/**
	 * Create a field relative pose3d from a given pose3d
	 * 
	 * @param robotRelativePose the pose3d of the object RELATIVE to the robot
	 * @param robotPose         the robot pose
	 * @return field relative pose3d of the object
	 */
	public static Pose3d fieldRelativePose3d(Pose3d robotRelativePose,
			Pose2d robotPose) {
		return new Pose3d(robotPose)
				.transformBy(new Transform3d(robotRelativePose.getTranslation(),
						robotRelativePose.getRotation()));
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.none();
	}

	@Override
	public double getCurrent() {
		return 0;
	}

	@Override
	public HashMap<String, Double> getTemps() {
		return new HashMap<>(Map.of("NULL", 0.0));
	}

	@Override
	public void setCurrentLimit(int amps) {
		return;
	}
}
