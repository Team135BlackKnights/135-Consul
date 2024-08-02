package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.PVCameras;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants;

/**
 * Contains all cameras and estimators for the robot, and updates the vision
 * (simulation or real) based on the robot's current pose (either expected or
 * physics result).
 */
public class VisionIOPhotonVision implements VisionIO {
	public static VisionSystemSim visionSim;
	//These estimate pose based on april tag location
	public final PhotonPoseEstimator rightEstimator, frontEstimator,
			leftEstimator, backEstimator;
	//Put these values into an array to iterate through them (go through them in order)
	public final PhotonPoseEstimator[] camEstimates;
	public final PhotonCamera[] cams;
	//Used for aligning with a particular aprilTag
	public static double camXError = 0f;
	static double distance = 0;
	private static PhotonCamera frontCam, rightCam, leftCam, backCam;

	public VisionIOPhotonVision() {
		//load the field 
		AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
				.loadField(AprilTagFields.k2024Crescendo);
		//Create the cameras
		rightCam = new PhotonCamera(VisionConstants.rightCamName);
		backCam = new PhotonCamera(VisionConstants.backCamName);
		frontCam = new PhotonCamera(VisionConstants.frontCamName);
		leftCam = new PhotonCamera(VisionConstants.leftCamName);
		//Set the pipelines (how it computes what we want), similar to limelight, default 0
		backCam.setPipelineIndex(0);
		rightCam.setPipelineIndex(0);
		frontCam.setPipelineIndex(0);
		leftCam.setPipelineIndex(0);
		//Create new photon pose estimators
		rightEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,
				VisionConstants.robotToRight);
		backEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
				VisionConstants.robotToBack);
		leftEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam,
				VisionConstants.robotToLeft);
		frontEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,
				VisionConstants.robotToFront);
		//Set the strategy to use when multitag isn't detected. IE, only one apriltag visible
		rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		//Puts the estimators in an array to iterate through them efficiently
		camEstimates = new PhotonPoseEstimator[] { frontEstimator, leftEstimator,
				rightEstimator, backEstimator
		};
		//Same process for cameras
		cams = new PhotonCamera[] { frontCam, leftCam, rightCam, backCam
		};
		//Create PhotonVision camera simulations if the robot is in sim
		if (Constants.currentMode == Mode.SIM) {
			//Array for iteration
			PhotonCameraSim[] cameraSims = new PhotonCameraSim[] {
					new PhotonCameraSim(frontCam), new PhotonCameraSim(leftCam),
					new PhotonCameraSim(rightCam), new PhotonCameraSim(backCam)
			};
			//Handles the vision simulation
			visionSim = new VisionSystemSim("Vision Sim");
			//Adds the april tags
			visionSim.addAprilTags(fieldLayout);
			/*We use ARDUCAM OV9281s, which are the same cameras as on the limelight 3G. BLACK AND WHITE 
			This means we can use the limelight properties. In an array for iteration.*/
			SimCameraProperties[] properties = new SimCameraProperties[] {
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
			};
			//Adds each camera present in the previous array to the visionSim. 
			for (var i = 0; i < cams.length; i++) {
				cameraSims[i] = new PhotonCameraSim(cams[i], properties[i]);
				visionSim.addCamera(cameraSims[i],
						VisionConstants.camTranslations[i]);
				//the following are cool to see in sim, but take over 80ms to run, so use at risk for debugging
				cameraSims[i].enableRawStream(true);
				cameraSims[i].enableProcessedStream(true);
				cameraSims[i].enableDrawWireframe(true);
			}
		}
	}

	@Override
	public void updateVisionSim(String objectName, Pose2d estimatedPose) {
		visionSim.getDebugField().getObject(objectName).setPose(estimatedPose);
	}

	@Override
	public void updateVisionObject(String objectName) {
		visionSim.getDebugField().getObject(objectName).setPoses();
	}

	public double[] getEstimationFactors(Pose2d estimatedPose,
			PhotonPoseEstimator photonEstimator, PhotonCamera cam,
			List<Integer> numTags) {
		//Default Std deviation
		//All the targets that have been pulled from a camera
		var targets = cam.getLatestResult().getTargets();
		//Number of tags and average distance
		double avgDist = 0, avgPoseAmbiguity = 0;
		double lowestDist = Double.MAX_VALUE;
		//Iterate through each target
		for (var tgt : targets) {
			//Return the distance from the individual tag and the number of tags
			var tagPose = photonEstimator.getFieldTags()
					.getTagPose(tgt.getFiducialId());
			if (tagPose.isEmpty())
				continue;
			double dist = tagPose.get().toPose2d().getTranslation()
					.getDistance(estimatedPose.getTranslation());
			if (dist < lowestDist) {
				lowestDist = dist;
			}
			avgDist += dist;
			avgPoseAmbiguity += tgt.getPoseAmbiguity();
		}
		avgDist /= numTags.size();
		avgPoseAmbiguity /= numTags.size();
		double weighAverage = 0;
		for (int tag : numTags) {
			weighAverage += VisionConstants.FieldConstants.aprilTagOffsets[tag];
		}
		weighAverage /= numTags.size();
		return new double[] { avgDist, lowestDist, weighAverage, avgPoseAmbiguity
		};
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		//If code's in sim update the simulated pose estimator
		if (Constants.currentMode == Mode.SIM) {
			visionSim.update(RobotContainer.fieldSimulation
					.getSwerveDriveSimulation().getPose3d().toPose2d());
		}
		//Update the global pose for each camera
		for (int i = 0; i < cams.length; i++) {
			PhotonPoseEstimator cEstimator = camEstimates[i];
			PhotonCamera cCam = cams[i];
			if (cCam.isConnected()) {
				var visionEst = getEstimatedGlobalPose(cEstimator, cCam,
						RobotContainer.drivetrainS.getPose());
				final int index = i; // Create a final copy of the loop variable
				PVCameras camera = PVCameras.getCameraByIndex(index);
				visionEst.ifPresentOrElse(est -> {
					Pose2d estPose = est.estimatedPose.toPose2d();
					List<Integer> aprilTagList = new ArrayList<>();
					for (PhotonTrackedTarget target : est.targetsUsed)
						aprilTagList.add(target.getFiducialId());
					/*for (PhotonTrackedTarget target : est.targetsUsed) { //To test the Offset feature, uncomment this.
						if (target.getFiducialId() == 6) {
							estPose = estPose.div(4012);
							//estPose = new Pose2d(0, 0, new Rotation2d(0,0));
						}
					}*/
					int[] aprilTags = new int[aprilTagList.size()];
					for (int j = 0; j < aprilTagList.size(); j++) {
						aprilTags[j] = aprilTagList.get(j);
					}
					inputs.time[index] = est.timestampSeconds;
					double[] factors = getEstimationFactors(estPose, cEstimator,
							cCam, aprilTagList);
					inputs.avgDist[index] = factors[0];
					inputs.lowestDist[index] = factors[1];
					inputs.weightAverage[index] = factors[2];
					inputs.avgPoseAmbiguity[index] = factors[3];
					inputs.estPose[index] = estPose;
					switch (camera) {
					case Front_Camera:
						inputs.frontCamTagList = aprilTags;
						inputs.frontCamHeartbeat = true;
						break;
					case Left_Camera:
						inputs.leftCamTagList = aprilTags;
						inputs.leftCamHeartbeat = true;
						break;
					case Right_Camera:
						inputs.rightCamTagList = aprilTags;
						inputs.rightCamHeartbeat = true;
						break;
					case Back_Camera:
						inputs.backCamTagList = aprilTags;
						inputs.backCamHeartbeat = true;
						break;
					}
				}, () -> {
					inputs.estPose[index] = new Pose2d();
					inputs.avgDist[index] = 0;
					inputs.lowestDist[index] = 0;
					inputs.weightAverage[index] = 0;
					inputs.avgPoseAmbiguity[index] = 0;
					switch (camera) {
					case Front_Camera:
						inputs.frontCamTagList = new int[0];
						inputs.frontCamHeartbeat = true;
						break;
					case Left_Camera:
						inputs.leftCamTagList = new int[0];
						inputs.leftCamHeartbeat = true;
						break;
					case Right_Camera:
						inputs.rightCamTagList = new int[0];
						inputs.rightCamHeartbeat = true;
						break;
					case Back_Camera:
						inputs.backCamTagList = new int[0];
						inputs.backCamHeartbeat = true;
						break;
					}
					inputs.time[index] = 0;
				});
			} else {
				switch (PVCameras.getCameraByIndex(i)) {
				case Front_Camera:
					inputs.frontCamHeartbeat = false;
					break;
				case Left_Camera:
					inputs.leftCamHeartbeat = false;
					break;
				case Right_Camera:
					inputs.rightCamHeartbeat = false;
					break;
				case Back_Camera:
					inputs.backCamHeartbeat = false;
					break;
				}
			}
		}
		inputs.aprilTagOffsets = VisionConstants.FieldConstants.aprilTagOffsets;
	}

	/**
	 * Uses one particular camera to figure out if the AprilTags in the argument
	 * is within sight or not, then returning the Pose3d if found. ROBOT
	 * RELATIVE.
	 * 
	 * @param Camera       the camera to use
	 * @param tagToLookFor the potential aprilTag to look for
	 * @return An optional Pose3d which is ROBOT RELATIVE to the selected tag.
	 */
	public static Optional<Pose2d> aprilTagPose2d(PhotonCamera camera,
			int tagToLookFor) {
		var targets = camera.getLatestResult().getTargets();
		@SuppressWarnings("unlikely-arg-type")
		Optional<PhotonTrackedTarget> foundTargets = targets.stream()
				.filter(t -> t.getFiducialId() == tagToLookFor)
				.filter(t -> !t.equals(tagToLookFor) && t.getPoseAmbiguity() <= .8
						&& t.getPoseAmbiguity() != -1)
				.findFirst();
		if (foundTargets.isPresent()) {
			Transform3d cameraToTarget = foundTargets.get()
					.getBestCameraToTarget();
			return Optional
					.of(new Pose2d(cameraToTarget.getTranslation().toTranslation2d(),
							cameraToTarget.getRotation().toRotation2d()));
		}
		return Optional.empty();
	}

	/**
	 * @param camera
	 * @return
	 */
	public static boolean aprilTagVisible(PhotonCamera camera) {
		return !camera.getLatestResult().getTargets().isEmpty();
	}

	/**
	 * Estimates the simulator global pose based on an individual photon
	 * estimator and updates the sim field if its in simulation
	 * 
	 * @param photonEstimator the estimator to use
	 * @param camera          the camera to use
	 * @param prevEstPose2d   the previous pose of the estimator
	 * @return the pose estimator (if it exists)
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
			PhotonPoseEstimator photonEstimator, PhotonCamera camera,
			Pose2d prevEstPose2d) {
		//Feed it previous pose
		photonEstimator.setReferencePose(prevEstPose2d);
		//Update the estimated position
		var visionEst = photonEstimator.update();
		//Return the timestamp, check results
		boolean newResult = false;
		//Figure out what camera it is
		PVCameras cameraVal;
		switch (camera.getName()) {
		case VisionConstants.frontCamName:
			cameraVal = PVCameras.Front_Camera;
			break;
		case VisionConstants.leftCamName:
			cameraVal = PVCameras.Left_Camera;
			break;
		case VisionConstants.rightCamName:
			cameraVal = PVCameras.Right_Camera;
			break;
		//By elimination it means that this becomes back camera
		default:
			cameraVal = PVCameras.Back_Camera;
			break;
		}
		if (Robot.isSimulation()) {
			updateSimField(visionEst, newResult, cameraVal);
		}
		return visionEst;
	}

	/**
	 * If the robot is in simulation, updates the states of the simulated camera
	 * and the simulated robot pose
	 * 
	 * @param visionEst The output from the getPhotonPoseEstimate()
	 * @param newResult Checks to see whether to set them to the field
	 * @param camera    The camera to use, as a custom PVCam enum declared in
	 *                     visionConstants
	 */
	private void updateSimField(Optional<EstimatedRobotPose> visionEst,
			boolean newResult, VisionConstants.PVCameras camera) {
		if (Constants.currentMode == Mode.SIM) {
			//Names the object based on the camera
			String objectName;
			switch (camera) {
			case Front_Camera:
				objectName = "VisionEstimationF";
				break;
			case Left_Camera:
				objectName = "VisionEstimationL";
				break;
			case Right_Camera:
				objectName = "VisionEstimationR";
				break;
			default:
				objectName = "VisionEstimationB";
				break;
			}
			visionEst.ifPresentOrElse(
					est -> updateVisionSim(objectName, est.estimatedPose.toPose2d()),
					() -> {
						if (newResult)
							updateVisionObject(objectName);
					});
		}
	}
}
