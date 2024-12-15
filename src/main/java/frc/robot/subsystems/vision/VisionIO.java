package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
	@AutoLog
	public static class VisionIOInputs {
	  public boolean connected = false;
	  public String name = "";
	  public TargetObservation latestTargetObservation =
		  new TargetObservation(new Rotation2d(), new Rotation2d(), -1);
	  public PoseObservation[] poseObservations = new PoseObservation[0];
	  public int[] tagIds = new int[0];
	}
  
	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty, int id) {}
  
	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(
		double timestamp,
		Pose3d pose,
		double ambiguity,
		int tagCount,
		double averageTagDistance,
		PoseObservationType type) {}
  
	public static enum PoseObservationType {
	  PHOTONVISION
	  //QuestNav?
	}
	public static enum CameraID{
		FRONT_LEFT,
		FRONT_RIGHT,
		BACK_LEFT,
		BACK_RIGHT
	}
	public default void updateInputs(VisionIOInputs inputs) {}
  }