package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    // Common inputs
    public boolean connected = false;
    public boolean ntConnected = false;
    public String name = "";
    
    // PhotonVision-style inputs
    public TargetObservation[] targetObservations = new TargetObservation[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }
  
  @AutoLog
  public static class AprilTagVisionIOInputs {
    // Northstar-style AprilTag inputs
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }
  
  @AutoLog
  public static class ObjDetectVisionIOInputs {
    // Northstar-style object detection inputs
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;
  }

  /** Represents the position of a simple target, not used for pose estimation. */
  public static record TargetObservation(
      Rotation2d tx, 
      Rotation2d ty, 
      int id, 
      Transform3d cameraToTarget, 
      double timestamp) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION,
    NORTHSTAR
  }
  
  public static enum CameraID {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
  
  /** 
   * Update inputs for PhotonVision-style cameras.
   * Default implementation does nothing.
   */
  public default void updateInputs(VisionIOInputs inputs) {}
  
  /**
   * Update inputs for Northstar-style cameras.
   * Default implementation does nothing.
   */
  public default void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {}
  
  /** 
   * Set recording state for Northstar cameras.
   * Default implementation does nothing.
   */
  public default void setRecording(boolean active) {}
}