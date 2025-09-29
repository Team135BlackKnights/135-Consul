package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.vision.SelfCheckingLimelight;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.AITargets;

public class Vision extends SubsystemChecker {
	private final Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final AprilTagVisionIOInputsAutoLogged[] aprilTagInputs;
	private final ObjDetectVisionIOInputsAutoLogged[] objDetectInputs;
  private final LoggedNetworkBoolean recordingRequest =
      new LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);
	// Camera type tracking
	private final CameraType[] cameraTypes;

	private boolean staleReading = false;
	private Pose2d lastOdomPose = new Pose2d(0, 0, new Rotation2d(0));

	private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
	private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

	private final double disconnectedTimeout = 0.5;
	private final Timer[] disconnectedTimers;
	private final Alert[] disconnectedAlerts;

	public enum CameraType {
		PHOTONVISION,
		Southmoon
	}

	public Vision(Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier, VisionIO... io) {
		this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
		this.io = io;
		this.cameraTypes = new CameraType[io.length];

		inputs = new VisionIOInputsAutoLogged[io.length];
		aprilTagInputs = new AprilTagVisionIOInputsAutoLogged[io.length];
		objDetectInputs = new ObjDetectVisionIOInputsAutoLogged[io.length];
		disconnectedTimers = new Timer[io.length];
		disconnectedAlerts = new Alert[io.length];

		for (int i = 0; i < io.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
			aprilTagInputs[i] = new AprilTagVisionIOInputsAutoLogged();
			objDetectInputs[i] = new ObjDetectVisionIOInputsAutoLogged();
			disconnectedAlerts[i] = new Alert("", Alert.AlertType.kError);
			lastFrameTimes.put(i, 0.0);
			disconnectedTimers[i] = new Timer();
			disconnectedTimers[i].start();

			// Detect camera type
			cameraTypes[i] = io[i] instanceof VisionIOSouthmoon ? CameraType.Southmoon : CameraType.PHOTONVISION;
		}

		registerSelfCheckHardware();
	}

	@Override
	public void periodic() {
		long timestamp = System.currentTimeMillis();

		// Update all cameras
		for (int i = 0; i < io.length; i++) {
			if (cameraTypes[i] == CameraType.Southmoon) {
				io[i].updateInputs(inputs[i], aprilTagInputs[i], objDetectInputs[i]);
				Logger.processInputs("Vision/Southmoon" + i, inputs[i]);
				Logger.processInputs("Vision/Southmoon" + i + "/AprilTag", aprilTagInputs[i]);
				Logger.processInputs("Vision/Southmoon" + i + "/ObjDetect", objDetectInputs[i]);
			} else {
				io[i].updateInputs(inputs[i]);
				Logger.processInputs("Vision/Camera" + i, inputs[i]);
			}
		}

		Logger.recordOutput("SystemStatus/Periodic/VisionInputsMS", System.currentTimeMillis() - timestamp);

		// Update recording state for Southmoon cameras
		boolean shouldRecord = DriverStation.isFMSAttached() || recordingRequest.get();
		for (int i = 0; i < io.length; i++) {
			if (cameraTypes[i] == CameraType.Southmoon) {
				io[i].setRecording(shouldRecord);
			}
		}

		timestamp = System.currentTimeMillis();

		// Initialize logging values
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

		// Update disconnected alerts
		boolean anyNTDisconnected = false;
		for (int i = 0; i < io.length; i++) {
			boolean hasData = aprilTagInputs[i].timestamps.length > 0
					|| objDetectInputs[i].timestamps.length > 0
					|| inputs[i].poseObservations.length > 0;

			if (hasData) {
				disconnectedTimers[i].reset();
			}

			boolean disconnected = disconnectedTimers[i].hasElapsed(disconnectedTimeout)
					|| !inputs[i].connected;

			if (disconnected) {
				String cameraName = cameraTypes[i] == CameraType.Southmoon ? "Southmoon " + i : inputs[i].name;
				disconnectedAlerts[i].setText(
						inputs[i].connected ? cameraName + " connected but not publishing frames"
								: cameraName + " disconnected");
				addFault("NO heartbeat detected from " + cameraName);
			}
			disconnectedAlerts[i].set(disconnected);
			anyNTDisconnected = anyNTDisconnected || !inputs[i].connected;
		}

		// Process camera data based on type
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			if (cameraTypes[cameraIndex] == CameraType.PHOTONVISION) {
				processPhotonVisionCamera(cameraIndex, allTagPoses, allRobotPoses,
						allRobotPosesAccepted, allRobotPosesRejected);
			} else {
				processSouthmoonCamera(cameraIndex, allTagPoses, allRobotPoses,
						allRobotPosesAccepted, allRobotPosesRejected);
			}
		}

		lastOdomPose = currentOdomPose;

		// Log summary data
		Logger.recordOutput("Vision/Summary/TagPoses",
				allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
		Logger.recordOutput("Vision/Summary/RobotPoses",
				allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
		Logger.recordOutput("Vision/Summary/RobotPosesAccepted",
				allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
		Logger.recordOutput("Vision/Summary/RobotPosesRejected",
				allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
		Logger.recordOutput("Vision/FieldTrusts", VisionConstants.FieldConstants.aprilTagOffsets);
		Logger.recordOutput("SystemStatus/Periodic/VisionProcessMS",
				System.currentTimeMillis() - timestamp);
	}

	private void processPhotonVisionCamera(int cameraIndex, List<Pose3d> allTagPoses,
			List<Pose3d> allRobotPoses, List<Pose3d> allRobotPosesAccepted,
			List<Pose3d> allRobotPosesRejected) {

		List<Pose3d> tagPoses = new LinkedList<>();
		List<Pose3d> robotPoses = new LinkedList<>();
		List<Pose3d> robotPosesAccepted = new LinkedList<>();
		List<Pose3d> robotPosesRejected = new LinkedList<>();
		double averageTrust = 0.0;

		// Add tag poses
		for (int tagId : inputs[cameraIndex].tagIds) {
			var tagPose = aprilTagLayoutSupplier.get().getLayout().getTagPose(tagId);
			if (tagPose.isPresent()) {
				tagPoses.add(tagPose.get());
				lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
			}
			averageTrust += VisionConstants.FieldConstants.aprilTagOffsets[tagId];
		}
		if (inputs[cameraIndex].tagIds.length > 0) {
			averageTrust /= inputs[cameraIndex].tagIds.length;
		}

		// Loop over pose observations
		for (var observation : inputs[cameraIndex].poseObservations) {
			boolean rejectPose = shouldRejectPose(observation, averageTrust);

			robotPoses.add(observation.pose());
			if (rejectPose) {
				robotPosesRejected.add(observation.pose());
				if (!staleReading) {
					updateTagTrust(inputs[cameraIndex].tagIds, true);
				}
			} else {
				robotPosesAccepted.add(observation.pose());

				// Calculate standard deviations
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 1.0)
						/ observation.tagCount();
				double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
				double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
				linearStdDev *= averageTrust;
				angularStdDev *= averageTrust;

				// Send vision observation
				addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

				if (!staleReading) {
					updateTagTrust(inputs[cameraIndex].tagIds, false);
				}
			}
		}

		// Log camera data
		String cameraName = inputs[cameraIndex].name;
		Logger.recordOutput("Vision/" + cameraName + "/TagPoses",
				tagPoses.toArray(new Pose3d[tagPoses.size()]));
		Logger.recordOutput("Vision/" + cameraName + "/RobotPoses",
				robotPoses.toArray(new Pose3d[robotPoses.size()]));
		Logger.recordOutput("Vision/" + cameraName + "/RobotPosesAccepted",
				robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
		Logger.recordOutput("Vision/" + cameraName + "/RobotPosesRejected",
				robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

		allTagPoses.addAll(tagPoses);
		allRobotPoses.addAll(robotPoses);
		allRobotPosesAccepted.addAll(robotPosesAccepted);
		allRobotPosesRejected.addAll(robotPosesRejected);
	}

	private void processSouthmoonCamera(
        int cameraIndex,
        List<Pose3d> allTagPoses,
        List<Pose3d> allRobotPoses,
        List<Pose3d> allRobotPosesAccepted,
        List<Pose3d> allRobotPosesRejected) {

    // === APRILTAG POSE DETECTION ===
    for (int frameIndex = 0; frameIndex < aprilTagInputs[cameraIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(cameraIndex, Timer.getFPGATimestamp());
        double timestamp = aprilTagInputs[cameraIndex].timestamps[frameIndex];
        double[] values = aprilTagInputs[cameraIndex].frames[frameIndex];

        // Skip blank frame
        if (values.length == 0 || values[0] == 0) continue;

        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        boolean useVisionRotation = false;

        switch ((int) values[0]) {
            case 1 -> {
                // One pose (multi-tag)
                cameraPose = new Pose3d(
                        values[2], values[3], values[4],
                        new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[5], values[6], values[7], values[8])));
                robotPose = cameraPose.toPose2d()
                        .transformBy(GeomUtil.poseToTransform(VisionConstants.cameras[cameraIndex].getPose().get().toPose2d()).inverse());
                useVisionRotation = true;
            }
            case 2 -> {
                // Two poses (single tag, ambiguous)
                double error0 = values[1];
                double error1 = values[9];
                Pose3d cameraPose0 = new Pose3d(
                        values[2], values[3], values[4],
                        new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[5], values[6], values[7], values[8])));
                Pose3d cameraPose1 = new Pose3d(
                        values[10], values[11], values[12],
                        new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[13], values[14], values[15], values[16])));
                Transform2d cameraToRobot = GeomUtil.poseToTransform(VisionConstants.cameras[cameraIndex].getPose().get().toPose2d()).inverse();
                Pose2d robotPose0 = cameraPose0.toPose2d().transformBy(cameraToRobot);
                Pose2d robotPose1 = cameraPose1.toPose2d().transformBy(cameraToRobot);

                // Select disambiguated pose
                if (error0 < error1 * VisionConstants.ambiguityThreshold ||
                    error1 < error0 * VisionConstants.ambiguityThreshold) {
                    Rotation2d currentRotation = RobotContainer.drivetrainS.getPose().getRotation();
                    if (Math.abs(currentRotation.minus(robotPose0.getRotation()).getRadians())
                        < Math.abs(currentRotation.minus(robotPose1.getRotation()).getRadians())) {
                        cameraPose = cameraPose0;
                        robotPose = robotPose0;
                    } else {
                        cameraPose = cameraPose1;
                        robotPose = robotPose1;
                    }
                }
            }
        }

        if (cameraPose == null || robotPose == null) continue;

        // Reject off-field
		if (robotPose.getX() < 0
            || robotPose.getX() > aprilTagLayoutSupplier.get().getLayout().getFieldLength()
            || robotPose.getY() < 0
            || robotPose.getY() > aprilTagLayoutSupplier.get().getLayout().getFieldWidth()) {
            continue;
        }

        // Collect tag poses
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
            int tagId = (int) values[i];
            lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
            aprilTagLayoutSupplier.get().getLayout().getTagPose(tagId).ifPresent(tagPoses::add);
        }
        if (tagPoses.isEmpty()) continue;

        // Average distance to tags
        double totalDist = 0.0;
        for (Pose3d tagPose : tagPoses) {
            totalDist += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDist / tagPoses.size();

        // Standard deviations
        double xyStdDev = VisionConstants.linearStdDevBaseline
                * Math.pow(avgDistance, 1.2) / Math.pow(tagPoses.size(), 2.0);
        double thetaStdDev = useVisionRotation
                ? VisionConstants.angularStdDevBaseline
                    * Math.pow(avgDistance, 1.2) / Math.pow(tagPoses.size(), 2.0)
                : Double.POSITIVE_INFINITY;

        // Add measurement
        allRobotPoses.add(new Pose3d(robotPose));
        addVisionMeasurement(robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
        allRobotPosesAccepted.add(new Pose3d(robotPose));
        allTagPoses.addAll(tagPoses);

        // Logging
        Logger.recordOutput("Vision/Southmoon" + cameraIndex + "/RobotPose", robotPose);
        Logger.recordOutput("Vision/Southmoon" + cameraIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
    }

    // === OBJECT DETECTION ===
    for (int frameIndex = 0; frameIndex < objDetectInputs[cameraIndex].timestamps.length; frameIndex++) {
        double timestamp = objDetectInputs[cameraIndex].timestamps[frameIndex];
        double[] frame = objDetectInputs[cameraIndex].frames[frameIndex];

        for (int i = 0; i < frame.length; i += 10) {
            int classId = (int) frame[i];
            double confidence = frame[i + 1];

            if (confidence < VisionConstants.objDetectConfidenceThreshold) continue;

            double[] tx = new double[4];
            double[] ty = new double[4];
            for (int z = 0; z < 4; z++) {
                tx[z] = frame[i + 2 + (2 * z)];
                ty[z] = frame[i + 2 + (2 * z) + 1];
            }
			//TODO use usz

            Logger.recordOutput("Vision/Southmoon" + cameraIndex + "/DetectedObj" + classId + "/tx", tx);
            Logger.recordOutput("Vision/Southmoon" + cameraIndex + "/DetectedObj" + classId + "/ty", ty);
            Logger.recordOutput("Vision/Southmoon" + cameraIndex + "/DetectedObj" + classId + "/conf", confidence);
        }
    }
}


	/**
	 * Check if a pose observation should be rejected
	 */
	private boolean shouldRejectPose(PoseObservation observation, double averageTrust) {
		return observation.tagCount() == 0
				|| (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.ambiguityThreshold)
				|| Math.abs(observation.pose().getZ()) > VisionConstants.maxZError
				|| Math.abs(observation.pose().getRotation().toRotation2d().getDegrees()
						- RobotContainer.drivetrainS.getPose().getRotation().getDegrees()) > VisionConstants.maxYawError
				|| averageTrust < VisionConstants.FieldConstants.kFieldTagMinTrust
				|| observation.pose().getX() < 0.0
				|| observation.pose().getX() > aprilTagLayoutSupplier.get().getLayout().getFieldLength()
				|| observation.pose().getY() < 0.0
				|| observation.pose().getY() > aprilTagLayoutSupplier.get().getLayout().getFieldWidth();
	}

	/**
	 * Update tag trust values based on acceptance/rejection
	 */
	private void updateTagTrust(int[] tagIds, boolean rejected) {
		for (int tag : tagIds) {
			if (rejected) {
				VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math.min(10,
						VisionConstants.FieldConstants.aprilTagOffsets[tag] + .002);
			} else {
				VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math.max(1,
						VisionConstants.FieldConstants.aprilTagOffsets[tag] - .002);
			}
		}
	}

	static int counter = 0;
	static double lastTx = 0, lastTy = 0;
	static boolean noteDetected = false;

	public static Translation2d updateNotePose() {
		double gamePieceTx = 0, gamePieceTy = 0;
		Pose2d currentPose = RobotContainer.drivetrainS.getPose();
		if (Constants.currentMode == Mode.SIM) {
			// In simulation, get the current pose, and set the degree value to
			Translation2d targetPieceLocation = RobotContainer.fieldSimulation
					.getClosestGamePieceOnGround().getPose3d().toPose2d()
					.getTranslation();
			Logger.recordOutput("ClosestGamePiece", targetPieceLocation);
			double deltaX = targetPieceLocation.getX() - currentPose.getX();
			double deltaY = targetPieceLocation.getY() - currentPose.getY();
			gamePieceTx = Units.radiansToDegrees(Math.atan2(deltaY, deltaX)); // Use atan2 instead of atan
			gamePieceTx -= currentPose.getRotation().getDegrees();
			gamePieceTx = GeomUtil.closerAngleToZero(Rotation2d.fromDegrees(gamePieceTx));
			double d = currentPose.getTranslation()
					.getDistance(targetPieceLocation);
			double tyRad = Math.PI
					- Units.degreesToRadians(
							VisionConstants.limeLightAngleOffsetDegrees)
					- (Math.PI * 0.5D - Math.atan(d / Units.inchesToMeters(
							VisionConstants.limelightLensHeightoffFloorInches)));
			gamePieceTy = Units.radiansToDegrees(tyRad);
			noteDetected = true;
		} else {
			// THESE ARE IN D E G R E E S
			LimelightHelpers.LimelightTarget_Detector[] results = LimelightHelpers
					.getLatestResults(
							VisionConstants.limelightName).targetingResults.targets_Detector;
			System.out.println(results);
			for (LimelightHelpers.LimelightTarget_Detector object : results) {
				if (object.confidence < .4) {
					continue;
				}
				if (object.classID == AITargets.kGamePiece.getValue()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					noteDetected = true;
					Logger.recordOutput("Vision/NoteDetected", true);
				} else {
					noteDetected = false;
					Logger.recordOutput("Vision/NoteDetected", false);
				}
			}
		}
		if (gamePieceTx == lastTx && gamePieceTy == lastTy) {
			counter++;
		} else {
			counter = 0;
		}
		if (counter > 10) { // prevent flickering of note detection
			noteDetected = false;
		}
		if (noteDetected)
			return GeomUtil
					.calculateFieldRelativePose3d(currentPose, gamePieceTx, gamePieceTy,
							Units.inchesToMeters(
									VisionConstants.limelightLensHeightoffFloorInches),
							Units.inchesToMeters(2),
							VisionConstants.limeLightAngleOffsetDegrees)
					.getTranslation().toTranslation2d();
		return null;
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
	 * Calculates the error from the apriltag translation to the KNOWN translation
	 * of the apriltag.
	 * 
	 */
	public void poseErrorApproximation(int apriltagID) {
		TargetObservation[] observations = getLatestTargetObservations();
		Pose3d targetPose = null;
		boolean hasTarget = false;
		for (int i = 0; i < observations.length; i++) {
			if (observations[i].id() == apriltagID
					&& Math.abs(TimeUtil.getLogTimeSeconds() - observations[i].timestamp()) < 1) {
				Pose3d cameraPose = VisionConstants.cameras[i].getPose().get();
				Pose3d fieldToCameraPose = new Pose3d(RobotContainer.drivetrainS.getPose())
						.transformBy(
								new Transform3d(cameraPose.getTranslation().getX(), cameraPose.getTranslation().getY(),
										cameraPose.getTranslation().getZ(), cameraPose.getRotation()));
				Pose3d fieldToTagPose = fieldToCameraPose.transformBy(observations[i].cameraToTarget());
				targetPose = fieldToTagPose;
				hasTarget = true;
				Logger.recordOutput("Vision/poseErrorTagPose" + apriltagID, targetPose);
				break;
			}
		}
		if (!hasTarget) {
			Logger.recordOutput("Vision/poseError" + apriltagID, Double.NaN);
		} else {
			Pose3d knownPose = aprilTagLayoutSupplier.get().getLayout().getTagPose(apriltagID).get();
			double error = knownPose.getTranslation().getDistance(targetPose.getTranslation());
			Logger.recordOutput("Vision/poseError" + apriltagID, error);
		}
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
				inputs[cam.ordinal()].targetObservations[0].ty(), inputs[cam.ordinal()].targetObservations[0].id(),
				inputs[cam.ordinal()].targetObservations[0].cameraToTarget(),
				inputs[cam.ordinal()].targetObservations[0].timestamp());
	}

	/**
	 * Get all latest target observations from the photon vision cameras
	 * 
	 * @return an array of target observations, in the order of the CameraID enum
	 */
	public TargetObservation[] getLatestTargetObservations() {
		TargetObservation[] observations = new TargetObservation[CameraID.values().length];
		for (CameraID cam : CameraID.values()) {
			observations[cam.ordinal()] = getLatestTargetObservation(cam);
		}
		return observations;
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
