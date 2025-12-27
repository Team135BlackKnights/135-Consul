package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.FastSwerve.Swerve.TxTyObservation;
import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulation.CompetitionFieldSimulation;
import frc.robot.utils.drive.DriveConstants;
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
	private final LoggedNetworkBoolean recordingRequest = new LoggedNetworkBoolean("/SmartDashboard/Enable Recording",
			false);
	// Camera type tracking
	private final CameraType[] cameraTypes;

	private boolean staleReading = false;
	private Pose2d lastOdomPose = new Pose2d(0, 0, new Rotation2d(0));

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
		disconnectedTimers = new Timer[io.length];
		disconnectedAlerts = new Alert[io.length];

		for (int i = 0; i < io.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
			disconnectedAlerts[i] = new Alert("", Alert.AlertType.kError);
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

			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + inputs[i].name, inputs[i]);
			// turn this on for debugging camera positions
			Logger.recordOutput("Vision/" + inputs[i].name + "/CamPose",
					new Pose3d(RobotContainer.drivetrainS.getPose())
							.plus(GeomUtil.poseToTransform(VisionConstants.cameras[i].getPose().get())));

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
			boolean hasData = inputs[i].timestamps_april.length > 0
					|| inputs[i].timestamps_obj.length > 0
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
		Map<String, TxTyObservation> allTxTyObservations = new HashMap<>();

		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			if (cameraTypes[cameraIndex] == CameraType.PHOTONVISION) {
				processPhotonVisionCamera(cameraIndex, allTagPoses, allRobotPoses,
						allRobotPosesAccepted, allRobotPosesRejected);
				// TODO add tx ty obs for photonvision? unnecessary?
			} else {
				allTxTyObservations = processSouthmoonCamera(cameraIndex, allTagPoses, allRobotPoses,
						allRobotPosesAccepted, allRobotPosesRejected, allTxTyObservations);
			}
		}
		allTxTyObservations.values().stream().forEach((obs) -> RobotContainer.drivetrainS.addTxTyObservation(obs));
		if (Constants.currentMode == Mode.SIM) {
			// Pose2d simedAIPose = new Pose2d(2,2,Rotation2d.fromDegrees(0));
			// grab opposting robot sim poses
			Pose2d simedAIPose = CompetitionFieldSimulation.getClosestRobotPose(currentOdomPose.getTranslation());
			Pose2d simedAICoralPose = CompetitionFieldSimulation.getClosestGamePiece(Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated.class, currentOdomPose.getTranslation());
			if (simedAIPose != null) {
				TxTyObservation simedAIObservation = new TxTyObservation(AITargets.BLUE_BOT.name(), 0, new double[4],
						new double[4],
						simedAIPose.getTranslation()
								.getDistance(RobotContainer.drivetrainS.getPose().getTranslation()),
						TimeUtil.getLogTimeSeconds(), Optional.of(new Pose3d(simedAIPose)));
				allTxTyObservations.put(AITargets.BLUE_BOT.name(), simedAIObservation);
				RobotContainer.drivetrainS
						.addTxTyObservation(simedAIObservation);
			}
			if (simedAICoralPose != null) {
				TxTyObservation simedAICoralObservation = new TxTyObservation("CORAL", 0, new double[4], new double[4],
						simedAICoralPose.getTranslation()
								.getDistance(RobotContainer.drivetrainS.getPose().getTranslation()),
						TimeUtil.getLogTimeSeconds(), Optional.of(new Pose3d(simedAICoralPose)));
				allTxTyObservations.put("CORAL", simedAICoralObservation);

				RobotContainer.drivetrainS
						.addTxTyObservation(simedAICoralObservation);
			}

		}
		lastOdomPose = currentOdomPose;
		// Lastly, update our Pathfinding dynamic obstacles. If we're out of date, clear
		// them.
		List<Pair<Translation2d, Translation2d>> dynamicObstacles = new ArrayList<>();
		for (TxTyObservation obs : allTxTyObservations.values()) {
			if ((obs.observationName().startsWith("BLUE_") || obs.observationName().startsWith("RED_")) // only AI bots
					&& obs.objectPose().isPresent()) {

				Pose2d obsPose2d = obs.objectPose().get().toPose2d();
				double theta = obsPose2d.getRotation().getRadians();

				Translation2d center = obsPose2d.getTranslation();

				Translation2d fwd = new Translation2d(Math.cos(theta), Math.sin(theta));
				Translation2d side = new Translation2d(-Math.sin(theta), Math.cos(theta));

				double halfL = DriveConstants.kBumperToBumperLength / 2.0;
				double halfW = DriveConstants.kBumperToBumperWidth / 2.0;

				Translation2d c1 = center.plus(fwd.times(halfL)).plus(side.times(halfW));
				Translation2d c2 = center.plus(fwd.times(halfL)).plus(side.times(-halfW));
				Translation2d c3 = center.plus(fwd.times(-halfL)).plus(side.times(halfW));
				Translation2d c4 = center.plus(fwd.times(-halfL)).plus(side.times(-halfW));

				// Reduce to AABB (min/max corners)
				double minX = Math.min(Math.min(c1.getX(), c2.getX()), Math.min(c3.getX(), c4.getX()));
				double maxX = Math.max(Math.max(c1.getX(), c2.getX()), Math.max(c3.getX(), c4.getX()));
				double minY = Math.min(Math.min(c1.getY(), c2.getY()), Math.min(c3.getY(), c4.getY()));
				double maxY = Math.max(Math.max(c1.getY(), c2.getY()), Math.max(c3.getY(), c4.getY()));

				dynamicObstacles.add(
						new Pair<>(new Translation2d(minX, minY), new Translation2d(maxX, maxY)));

				Logger.recordOutput("Vision/DynamicObstacle/" + obs.observationName() + "/Min",
						new Pose2d(new Translation2d(minX, minY), new Rotation2d()));
				Logger.recordOutput("Vision/DynamicObstacle/" + obs.observationName() + "/Max",
						new Pose2d(new Translation2d(maxX, maxY), new Rotation2d()));
			}
		}
		// System.out.println("Dynamic Obstacles: " + dynamicObstacles.size());
		RobotContainer.pathFinder.setDynamicObstacles(dynamicObstacles, currentOdomPose.getTranslation());

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
			}
			averageTrust += VisionConstants.FieldConstants.aprilTagOffsets[tagId - 1];
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

	private Map<String, TxTyObservation> processSouthmoonCamera(
			int cameraIndex,
			List<Pose3d> allTagPoses,
			List<Pose3d> allRobotPoses,
			List<Pose3d> allRobotPosesAccepted,
			List<Pose3d> allRobotPosesRejected, Map<String, TxTyObservation> allTxTyObservations) {

		// === APRILTAG POSE DETECTION ===
		for (int frameIndex = 0; frameIndex < inputs[cameraIndex].timestamps_april.length; frameIndex++) {
			double timestamp = inputs[cameraIndex].timestamps_april[frameIndex];
			double[] values = inputs[cameraIndex].frames_april[frameIndex];
			// Skip blank frame
			if (values.length == 0 || values[0] == 0)
				continue;
			Pose3d cameraPose = null;
			Pose2d robotPose = null;
			boolean useVisionRotation = false;

			switch ((int) values[0]) {
				case 1 -> {
					// One pose (multi-tag)
					cameraPose = new Pose3d(
							values[2], values[3], values[4],
							new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[5], values[6], values[7],
									values[8])));
					robotPose = cameraPose.toPose2d()
							.transformBy(GeomUtil
									.poseToTransform(VisionConstants.cameras[cameraIndex].getPose().get().toPose2d())
									.inverse());
					useVisionRotation = true;
				}
				case 2 -> {
					// Two poses (single tag, ambiguous)
					double error0 = values[1];
					double error1 = values[9];
					Pose3d cameraPose0 = new Pose3d(
							values[2], values[3], values[4],
							new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[5], values[6], values[7],
									values[8])));
					Pose3d cameraPose1 = new Pose3d(
							values[10], values[11], values[12],
							new Rotation3d(new edu.wpi.first.math.geometry.Quaternion(values[13], values[14],
									values[15], values[16])));
					Transform2d cameraToRobot = GeomUtil
							.poseToTransform(VisionConstants.cameras[cameraIndex].getPose().get().toPose2d()).inverse();
					Pose2d robotPose0 = cameraPose0.toPose2d().transformBy(cameraToRobot);
					Pose2d robotPose1 = cameraPose1.toPose2d().transformBy(cameraToRobot);

					// Select disambiguated pose
					if (error0 < VisionConstants.ambiguityThreshold ||
							error1 < VisionConstants.ambiguityThreshold) {
						if (error0 < error1 / 2) {
							cameraPose = cameraPose0;
							robotPose = robotPose0;
						}
						if (error1 < error0 / 2) {
							cameraPose = cameraPose1;
							robotPose = robotPose1;
						}
						/*
						 * Rotation2d currentRotation =
						 * RobotContainer.drivetrainS.getPose().getRotation();
						 * if (Math.abs(currentRotation.minus(robotPose0.getRotation()).getRadians()) <
						 * Math
						 * .abs(currentRotation.minus(robotPose1.getRotation()).getRadians())) {
						 * cameraPose = cameraPose0;
						 * robotPose = robotPose0;
						 * } else {
						 * cameraPose = cameraPose1;
						 * robotPose = robotPose1;
						 * }
						 */
						// take the lower one
						if (error0 < error1) {
							cameraPose = cameraPose0;
							robotPose = robotPose0;
						} else {
							cameraPose = cameraPose1;
							robotPose = robotPose1;
						}
					}
				}
			}

			if (cameraPose == null || robotPose == null)
				continue;

			// Reject off-field
			if (robotPose.getX() < 0
					|| robotPose.getX() > aprilTagLayoutSupplier.get().getLayout().getFieldLength()
					|| robotPose.getY() < 0
					|| robotPose.getY() > aprilTagLayoutSupplier.get().getLayout().getFieldWidth()) {
				// continue;
			}

			// Collect tag poses
			List<Pose3d> tagPoses = new ArrayList<>();
			boolean containsDemo = false;
			for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
				int tagId = (int) values[i];
				if (tagId == 42) {
					containsDemo = true;
					tagPoses.add(new Pose3d()); // assume 0
				} else {
					aprilTagLayoutSupplier.get().getLayout().getTagPose(tagId).ifPresent(tagPoses::add);
				}
			}
			if (tagPoses.isEmpty())
				continue;

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
			if (!containsDemo)
				addVisionMeasurement(robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
			allRobotPosesAccepted.add(new Pose3d(robotPose));
			allTagPoses.addAll(tagPoses);

			// Logging
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/stdDev", xyStdDev);
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/thetaStdDev", thetaStdDev);
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/time", timestamp);
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/avgDistance", avgDistance);
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/RobotPose", robotPose);
			Logger.recordOutput("Vision/" + inputs[cameraIndex].name + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
		}
		// === APRILTAG TX/TY DETECTION ===
		Map<String, TxTyObservation> txTyObservations = new HashMap<>();
		for (int frameIndex = 0; frameIndex < inputs[cameraIndex].timestamps_april.length; frameIndex++) {
			var timestamp = inputs[cameraIndex].timestamps_april[frameIndex];
			var values = inputs[cameraIndex].frames_april[frameIndex];
			int tagEstimationDataEndIndex = switch ((int) values[0]) {
				default -> 0;
				case 1 -> 8;
				case 2 -> 16;
			};

			for (int index = tagEstimationDataEndIndex + 1; index < values.length; index += 10) {
				double[] tx = new double[4];
				double[] ty = new double[4];
				for (int i = 0; i < 4; i++) {
					tx[i] = values[index + 1 + (2 * i)];
					ty[i] = values[index + 1 + (2 * i) + 1];
				}
				int tagId = (int) values[index];
				double distance = values[index + 9];

				txTyObservations.put(
						"A" + String.valueOf(tagId), new TxTyObservation("A" + String.valueOf(tagId), cameraIndex, tx,
								ty, distance, timestamp, Optional.empty()));
			}
		}

		// Save tx ty observation data
		for (var observation : txTyObservations.values()) {
			if (!allTxTyObservations.containsKey(observation.observationName())
					|| observation.distance() < allTxTyObservations.get(observation.observationName()).distance()) {
				allTxTyObservations.put(observation.observationName(), observation);
			}
		}
		// === OBJECT DETECTION ===

		for (int frameIndex = 0; frameIndex < inputs[cameraIndex].timestamps_obj.length; frameIndex++) {
			double timestamp = inputs[cameraIndex].timestamps_obj[frameIndex];
			double[] frame = inputs[cameraIndex].frames_obj[frameIndex];
			for (int i = 0; i < frame.length; i += 27) {
				int classId = (int) frame[i];
				if (classId == -1) {
					double[] tx = new double[4];
					double[] ty = new double[4];
					Pose3d pose = new Pose3d(
							frame[i + 12], frame[i + 13], frame[i + 14],
							new Rotation3d(
									new edu.wpi.first.math.geometry.Quaternion(frame[i + 15], frame[i + 16],
											frame[i + 17],
											frame[i + 18])));
					Pose2d drivetrainPose = RobotContainer.drivetrainS.getPose();
					double distanceMag = pose.toPose2d().getTranslation()
							.getDistance(drivetrainPose.getTranslation());
					allTxTyObservations.put(
							"CORAL",
							new TxTyObservation("CORAL", cameraIndex, tx,
									ty, distanceMag, timestamp, Optional.of(pose)));
					continue; // done with the obv
				}
				double confidence = frame[i + 1];

				if (confidence < VisionConstants.objDetectConfidenceThreshold)
					continue;

				double[] tx = new double[4];
				double[] ty = new double[4];
				for (int z = 0; z < 4; z++) {
					tx[z] = frame[i + 2 + (2 * z)];
					ty[z] = frame[i + 2 + (2 * z) + 1];
				}
				double error1 = frame[i + 11];
				double error2 = frame[i + 19];
				Pose3d rawFirstPose = new Pose3d(
						frame[i + 12], frame[i + 13], frame[i + 14],
						new Rotation3d(
								new edu.wpi.first.math.geometry.Quaternion(frame[i + 15], frame[i + 16], frame[i + 17],
										frame[i + 18])));
				Pose3d rawSecondPose = new Pose3d(
						frame[i + 20], frame[i + 21], frame[i + 22],
						new Rotation3d(
								new edu.wpi.first.math.geometry.Quaternion(frame[i + 23], frame[i + 24], frame[i + 25],
										frame[i + 26])));
				Pose3d objectPoseFirst = rawFirstPose;
				Pose3d objectPoseSecond = rawSecondPose;
				Pose2d drivetrainPose = RobotContainer.drivetrainS.getPose();
				if (VisionConstants.bumperDetection) {
					Translation2d separationFirst = rawFirstPose.toPose2d().getTranslation()
							.minus(drivetrainPose.getTranslation());
					double bumperHalfExtent = Units.inchesToMeters(36);
					double xOffsetFirst = Math.abs(separationFirst.getX()) > 1e-3
							? Math.copySign(bumperHalfExtent, separationFirst.getX())
							: 0.0;
					double yOffsetFirst = Math.abs(separationFirst.getY()) > 1e-3
							? Math.copySign(bumperHalfExtent, separationFirst.getY())
							: 0.0;
					Translation2d separationSecond = rawSecondPose.toPose2d().getTranslation()
							.minus(drivetrainPose.getTranslation());
					double xOffsetSecond = Math.abs(separationSecond.getX()) > 1e-3
							? Math.copySign(bumperHalfExtent, separationSecond.getX())
							: 0.0;
					double yOffsetSecond = Math.abs(separationSecond.getY()) > 1e-3
							? Math.copySign(bumperHalfExtent, separationSecond.getY())
							: 0.0;
					objectPoseFirst = rawFirstPose
							.plus(new Transform3d(xOffsetFirst, yOffsetFirst, 0.0, new Rotation3d()));
					objectPoseSecond = rawSecondPose
							.plus(new Transform3d(xOffsetSecond, yOffsetSecond, 0.0, new Rotation3d()));
				}
				double distanceMagOne = objectPoseFirst.toPose2d().getTranslation()
						.getDistance(drivetrainPose.getTranslation());
				double distanceMagTwo = objectPoseSecond.toPose2d().getTranslation()
						.getDistance(drivetrainPose.getTranslation());
				// use the closer one if either is below 1m, otherwise, use lower error
				Pose3d objectPose = null;
				double distanceMag;
				if (classId == 0) {
					objectPose = objectPoseFirst;
					distanceMag = distanceMagOne;
					if (!objectPose.getTranslation().equals(Translation3d.kZero))
						allTxTyObservations.put("CORAL",
								new TxTyObservation("CORAL", cameraIndex, tx,
										ty, distanceMag, timestamp, Optional.of(objectPose)));
				} else {
					if (distanceMagOne < 1 || distanceMagTwo < 1) {
						if (distanceMagOne >= distanceMagTwo) {
							objectPose = objectPoseFirst;
							distanceMag = distanceMagOne;
						} else {
							objectPose = objectPoseSecond;
							distanceMag = distanceMagTwo;
						}
					} else {
						if (error1 <= error2) {
							objectPose = objectPoseFirst;
							distanceMag = distanceMagOne;
						} else {
							objectPose = objectPoseSecond;
							distanceMag = distanceMagTwo;
						}
					}
					allTxTyObservations.put(
							AITargets.values()[classId].name(),
							new TxTyObservation(AITargets.values()[classId].name(), cameraIndex, tx,
									ty, distanceMag, timestamp, Optional.of(objectPose)));
				}

			}
		}
		return allTxTyObservations;
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
				VisionConstants.FieldConstants.aprilTagOffsets[tag - 1] = Math.min(10,
						VisionConstants.FieldConstants.aprilTagOffsets[tag - 1] + .002);
			} else {
				VisionConstants.FieldConstants.aprilTagOffsets[tag - 1] = Math.max(1,
						VisionConstants.FieldConstants.aprilTagOffsets[tag - 1] - .002);
			}
		}
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
