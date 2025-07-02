package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BuildConstants;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.Swerve.TxTyObservation;
import frc.robot.subsystems.vision.VisionIO.CameraID;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePiece;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.GamePieceInSimulation;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.AITargets;

public class Vision extends SubsystemChecker {
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private boolean staleReading = false;

	public Vision(VisionIO... io) {
		this.io = io;
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		checkAprilTagFieldLayout();
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
		ChassisSpeeds currentVelocity = RobotContainer.drivetrainS.getChassisSpeeds();
		Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
		staleReading = Math.hypot(currentVelocity.vxMetersPerSecond,
				currentVelocity.vyMetersPerSecond) < VisionConstants.maxVelocity
				&& Math.abs(currentVelocity.omegaRadiansPerSecond) < VisionConstants.maxAngularVelocity;
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
			// Loop over TxTy observations
			for (var observation : inputs[cameraIndex].txTyObservations) {
				if (!allTxTyObservations.containsKey(observation.tagId())
						|| observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
					allTxTyObservations.put(observation.tagId(), observation);
				}
			}
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
		allTxTyObservations.values().stream().forEach(this::addTxTyMeasurement);
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

	 int counter = 0;
	 double lastTx = 0, lastTy = 0;
	 boolean gamePieceDetected = false;

	public Pair<GamePiece, Translation2d> updateNotePose() {
		double gamePieceTx = 0, gamePieceTy = 0;
		GamePiece gamePiece = GamePiece.ALGAE_BALL;
		Pose2d currentPose = RobotContainer.drivetrainS.getPose();
		if (Constants.currentMode == Mode.SIM) {
			// In simulation, get the current pose, and set the degree value to
			GamePieceInSimulation targetPiece = RobotContainer.fieldSimulation.getClosestGamePieceOnGround();
			Translation2d targetPieceLocation = targetPiece.getPose3d().toPose2d()
					.getTranslation();
			Logger.recordOutput("ClosestGamePiece", targetPieceLocation);
			if (targetPiece instanceof Reefscape2025FieldObjects.AlgaeBallOnFieldSimulated) {
				gamePiece = GamePiece.ALGAE_BALL;
			} else if (targetPiece instanceof Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated) {
				gamePiece = GamePiece.REEFSCAPE_CORAL;
			}
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
			gamePieceDetected = true;
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
				if (object.classID == AITargets.kGamePieceOne.getValue()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					gamePieceDetected = true;
					gamePiece = GamePiece.ALGAE_BALL;
					Logger.recordOutput("Vision/GamePieceDetected", true);
				} else if (object.classID == AITargets.kGamePieceTwo.getValue()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					gamePieceDetected = true;
					gamePiece = GamePiece.REEFSCAPE_CORAL;
					Logger.recordOutput("Vision/GamePieceDetected", true);
				} else {
					gamePieceDetected = false;
					Logger.recordOutput("Vision/GamePieceDetected", false);
				}
			}
		}
		if (gamePieceTx == lastTx && gamePieceTy == lastTy) {
			counter++;
		} else {
			counter = 0;
		}
		if (counter > 10) { // prevent flickering of note detection
			gamePieceDetected = false;
		}
		if (gamePieceDetected)
			return Pair.of(gamePiece, GeomUtil
					.calculateFieldRelativePose3d(currentPose, gamePieceTx, gamePieceTy,
							Units.inchesToMeters(
									VisionConstants.limelightLensHeightoffFloorInches),
							gamePiece == GamePiece.ALGAE_BALL ? FieldConstants.ALGAE_BALL_HEIGHT
									: FieldConstants.REEFSCAPE_CORAL_HEIGHT,
							VisionConstants.limeLightAngleOffsetDegrees)
					.getTranslation().toTranslation2d());
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

	public void addTxTyMeasurement(Swerve.TxTyObservation observation) {
		if (RobotContainer.drivetrainS instanceof Swerve) {
			((Swerve) RobotContainer.drivetrainS).addTxTyObservation(observation);
		} else {
			DriverStation.reportWarning("Trying to add TxTy Pose to tank or mechanum. NOT implemented.", false);
		}
	}

	public boolean objectVisionOkay() {
		return true;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(new ArrayList<SelfChecking>(
				List.of(/*new SelfCheckingLimelight(VisionConstants.limelightName)*/)));
	}

	/**
	 * Calculates the error from the apriltag translation to the KNOWN translation
	 * of the apriltag.
	 * 
	 */
	@SuppressWarnings("null")
	public void poseErrorApproximation(int apriltagID) {
		TargetObservation[] observations = getLatestTargetObservations();
		Pose3d targetPose = null;
		boolean hasTarget = false;
		for (int i = 0; i < observations.length; i++) {
			if (observations[i].id() == apriltagID
					&& Math.abs(TimeUtil.getLogTimeSeconds() - observations[i].timestamp()) < 1) {
				Pose3d fieldToCameraPose = new Pose3d(RobotContainer.drivetrainS.getPose())
						.transformBy(VisionConstants.robotToCameraTransforms[i]);
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
			Pose3d knownPose = VisionConstants.kTagLayout.getTagPose(apriltagID).get();
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
		return new HashMap<>(Map.of("NoMotorForSubsystemVision", 0.0));
	}

	@Override
	public void setCurrentLimit(int amps) {
		return;
	}

	@SuppressWarnings("all")
	public void checkAprilTagFieldLayout() {
		if (!VisionConstants.kAprilTagFieldLayout.toString().toUpperCase().contains("2025".toUpperCase())) {
			String aprilTagString = VisionConstants.kAprilTagFieldLayout.toString();
			DriverStation.reportWarning("April Tag layout is in year "
					+ aprilTagString.substring(aprilTagString.indexOf("2"), aprilTagString.indexOf("2") + 4)
					+ ", expected 2025.", false);
		} else if (Constants.isCompetition && BuildConstants.BUILD_UNIX_TIME < VisionConstants.WELDED_FIELD_DATE
				&& VisionConstants.kAprilTagFieldLayout.equals(AprilTagFields.k2025ReefscapeWelded)) {
			DriverStation.reportWarning(
					"April Tag layout may be incorrectly configured. Expected Andymark (since date is before), got Welded. Please confirm in VisionConstants.java.",
					false);
		}
		// At worlds, we will be using the Welded field layout
		else if (Constants.isCompetition && BuildConstants.BUILD_UNIX_TIME > VisionConstants.ANDYMARK_FIELD_DATE
				&& VisionConstants.kAprilTagFieldLayout.equals(AprilTagFields.k2025ReefscapeWelded)) {
			DriverStation.reportWarning(
					"April Tag layout may be incorrectly configured. Expected Welded (since date is during Worlds), got Andymark. Please confirm in VisionConstants.java.",
					false);
		} else if (!Constants.isCompetition
				&& VisionConstants.kAprilTagFieldLayout.equals(AprilTagFields.k2025ReefscapeWelded)) {
			DriverStation.reportWarning(
					"April Tag layout may be incorrectly configured. Expected Andymark (since not at competition), got Welded. Please confirm in VisionConstants.java.",
					false);
		} else {
			System.out.println(
					"April Tag layout is configured to expected value of " + VisionConstants.kAprilTagFieldLayout);
		}
	}
}
