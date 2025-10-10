package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.vision.VisionConstants;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator photonEstimator;
  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier,String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    photonEstimator = new PhotonPoseEstimator(aprilTagLayoutSupplier.get().getLayout(),PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT,robotToCamera);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long timestamp = System.currentTimeMillis();
    inputs.connected = camera.isConnected();
	  inputs.name = camera.getName();
    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<TargetObservation> targetObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        Optional<EstimatedRobotPose> visionEst = photonEstimator.update(result);
        if (visionEst.isPresent()) {
          var visionResult = visionEst.get();
          double ambiguity;
          if (visionResult.targetsUsed.size() > 1 && result.getMultiTagResult().isPresent()) {
            ambiguity = result.getMultiTagResult().get().estimatedPose.ambiguity;
          }else{
            ambiguity = result.getBestTarget().poseAmbiguity; 
          }
            // Calculate average tag distance
          double totalTagDistance = 0.0;
          
          for (var target : visionResult.targetsUsed) {
            targetObservations.add(
            new TargetObservation(
                Rotation2d.fromDegrees(target.getYaw()),
                Rotation2d.fromDegrees(target.getPitch()),
                target.getFiducialId(),
                target.getBestCameraToTarget(),
                TimeUtil.getRealTimeSeconds()));
            totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();		  
          }

          // Add tag IDs
          tagIds.addAll(visionResult.targetsUsed.stream().map(t -> (short)t.fiducialId).toList());
          // Add observation
          poseObservations.add(
              new PoseObservation(
                visionResult.timestampSeconds, // Timestamp
                visionResult.estimatedPose, // 3D pose estimate
                ambiguity, // Ambiguity
                  visionResult.targetsUsed.size(), // Tag count
                  totalTagDistance / visionResult.targetsUsed.size(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
    }
    

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }
    // Save target observations to inputs object
    inputs.targetObservations = new TargetObservation[targetObservations.size()];
    for (int i = 0; i < targetObservations.size(); i++) {
      inputs.targetObservations[i] = targetObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
    Logger.recordOutput("Vision/Periodic/"+camera.getName()+"InputMS", System.currentTimeMillis() - timestamp);
  }
}