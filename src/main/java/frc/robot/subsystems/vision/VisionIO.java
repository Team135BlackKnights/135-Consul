package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.vision.VisionConstants;

public interface VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		public double[] avgDist = { 0, 0, 0, 0
		};
		public double[] lowestDist = { 0, 0, 0, 0
		};
		public double[] weightAverage = { 0, 0, 0, 0
		};
		public double[] avgPoseAmbiguity = { 0, 0, 0, 0
		};
		public double[] time = { 0, 0, 0, 0
		};
		public double[] aprilTagOffsets = VisionConstants.FieldConstants.aprilTagOffsets;
		public int[] frontCamTagList = {};
		public int[] leftCamTagList = {};
		public int[] rightCamTagList = {};
		public int[] backCamTagList = {};
		public boolean frontCamHeartbeat = false;
		public boolean leftCamHeartbeat = false;
		public boolean rightCamHeartbeat = false;
		public boolean backCamHeartbeat = false;
		public Pose2d[] estPose = { new Pose2d(), new Pose2d(), new Pose2d(),
				new Pose2d()
		};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(VisionIOInputs inputs) {}

	public default void updateVisionSim(String objectName,
			Pose2d estimatedPose) {}

	public default void updateVisionObject(String objectName) {}
}
