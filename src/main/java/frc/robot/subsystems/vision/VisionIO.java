package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Hardware abstraction boundary for robot vision.
 *
 * <p>Each camera backend fills in {@link VisionIO.VisionIOInputs}; {@link Vision} owns the
 * filtering and decides which measurements reach the drivetrain. This keeps
 * PhotonVision, Southmoon, simulation, and future camera code from spreading
 * protocol-specific details through commands.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    // Common health/status. "connected" is the backend's usable status;
    // "ntConnected" is specifically whether the NT4 client is connected.
    public boolean connected = false;
    public boolean ntConnected = false;
    public String name = "";

    // PhotonVision-style inputs: already parsed into target and robot pose
    // observations by the camera-specific IO implementation.
    public TargetObservation[] targetObservations = new TargetObservation[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];

    // Southmoon raw NT packets, based on the old Northstar packet style.
    // timestamps_* are NT server timestamps in seconds. frames_april and
    // frames_obj are compact double-array protocols parsed in
    // Vision.processSouthmoonCamera().
    public double[] timestamps_april = new double[] {};
    public double[][] frames_april = new double[][] {};
    public long fps_april = 0;
    public double[] timestamps_obj = new double[] {};
    public double[][] frames_obj = new double[][] {};
    public long fps_obj = 0;
    // Preferred custom-object path: one record per detection with camera angles
    // and estimated ground distance, but no full field pose.
    public ObjDetectTxyObservation[] objDetectTxyObservations = new ObjDetectTxyObservation[0];
  }

  /**
   * Represents a single target seen by a camera.
   *
   * <p>tx is horizontal camera angle, ty is vertical camera angle, and
   * cameraToTarget is the 3D transform from the camera lens to the target.
   */
  public static record TargetObservation(
      Rotation2d tx,
      Rotation2d ty,
      int id,
      Transform3d cameraToTarget,
      double timestamp) {}

  /**
   * Represents a custom object detection with tx/ty only.
   *
   * <p>classId must match VisionConstants.AITargets order and the model's class
   * order. distanceMeters is estimated by the coprocessor from the detection and
   * camera intrinsics.
   */
  public static record ObjDetectTxyObservation(
      int classId,
      double confidence,
      Rotation2d tx,
      Rotation2d ty,
      double distanceMeters,
      double timestamp) {}

  /**
   * Represents a full robot pose sample used for drivetrain pose estimation.
   *
   * <p>ambiguity is especially important for single-tag solvePnP results, where
   * two physically plausible poses can exist.
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION,
    // Legacy name for the custom coprocessor packet style used by Southmoon.
    NORTHSTAR
  }

  public static enum CameraID {
    INTAKE_CAM,
    BACK_RIGHT,
    BACK_LEFT
  }

  /**
   * Update this camera's inputs. Implementations should drain their hardware/NT
   * queues here and leave all filtering decisions to Vision.
   * Default implementation does nothing.
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Set recording state for coprocessors that support on-device video capture.
   * Default implementation does nothing.
   */
  public default void setRecording(boolean active) {}
}
