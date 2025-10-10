package frc.robot.utils.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.RequiredArgsConstructor;
import frc.robot.utils.GeomUtil.ApproachDirection;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningConstants;

public class VisionConstants {
	public static final FieldType fieldType = FieldType.ANDYMARK;

	public enum AITargets {
		//Make SURE these are in ORDER of the actual classID.
		BLUE_BOT,RED_BOT

	}

	// Command specific constants // Aim To Pose
	public static final ApproachDirection aimToPoseApproachDirection = ApproachDirection.FRONT, // Drive And Aim At Pose public static final ApproachDirection
			driveAndAimAtPoseApproachDirection = ApproachDirection.FRONT; // Drive To AI
	public static final ApproachDirection driveToAITargetApproachDirection = ApproachDirection.FRONT;

	public static class FieldConstants {
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		public static double[] aprilTagOffsets = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	}

	public static final boolean debug = true;

	public static final double ambiguityThreshold = 0.3;
	public static final double objDetectConfidenceThreshold = .4;
	public static final double maxZError = 0.75;
	public static final double maxYawError = 5.0;
	public static final double linearStdDevBaseline = 0.005;
	public static final double angularStdDevBaseline = 0.04;

	public static final double limeLightAngleOffsetDegrees = -40.0;
	public static final double limelightLensHeightoffFloorInches = 22.5;
	public static final String limelightName = "limelight-swerve";

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingDistance = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingDistance",
			Units.feetToMeters(4),
			TuningConstants.isTuningVision);

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingAngle = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingAngle",
			Units.degreesToRadians(3),
			TuningConstants.isTuningVision);

	public static final LoggableTunedNumber limelightCloseEnoughToConsiderMissingTimeout = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingTimeout", 1, TuningConstants.isTuningVision);

	public static final double maxStaleReadingXMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingYMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingRotation = Units.degreesToRadians(2);

	public static final CameraConfig[] cameras = new CameraConfig[] {
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									Units.inchesToMeters(11.791),
									Units.inchesToMeters(-10.041),
									Units.inchesToMeters(7.967-.037),
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(-25),
											Math.toRadians(-45))))
					.id("SPCA2630 PC Camera:usb_05c8_0a00_001_001")
					.width(1600)
					.height(1304)
					.exposure(180)
					.saturation(50)
					.hue(0)
					.whiteBalance(2800)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
								Units.inchesToMeters(11.791),
								Units.inchesToMeters(10.041),
								Units.inchesToMeters(7.967-.037),
								new Rotation3d(
										Math.toRadians(0.0),
										Math.toRadians(-25),
										Math.toRadians(45))))
					.id("SPCA2630 PC Camera:usb_05c8_0a00_001_002")
					.width(1600)
					.height(1304)
					.exposure(180)
					.saturation(50)
					.hue(0)
					.whiteBalance(2800)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
					CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									Units.inchesToMeters(-11.791),
									Units.inchesToMeters(-10.041),
									Units.inchesToMeters(7.967-.037),
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(-25),
											Math.toRadians(-135))))
					.id("SPCA2630 PC Camera:usb_05c8_0a00_001_003")
					.width(1600)
					.height(1304)
					.exposure(180)
					.saturation(50)
					.hue(0)
					.whiteBalance(2800)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
								Units.inchesToMeters(-11.791),
								Units.inchesToMeters(10.041),
								Units.inchesToMeters(7.967-.037),
								new Rotation3d(
										Math.toRadians(0.0),
										Math.toRadians(-25),
										Math.toRadians(135))))
					.id("SPCA2630 PC Camera:usb_05c8_0a00_001_004")
					.width(1600)
					.height(1304)
					.exposure(180)
					.saturation(50)
					.hue(0)
					.whiteBalance(2800)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
	};

	@Builder
	@NoArgsConstructor
	@AllArgsConstructor
	@Getter
	public static class CameraConfig {
		private Supplier<Pose3d> pose;
		private String id;
		private int width;
		private int height;
		private int autoExposure;
		private int autoWhiteBalance;
		private double exposure;
		private int saturation;
		private int hue;
		private int whiteBalance;
		private double gain;
		private double denoise;
	}
	//Transforms for alternative functions (like aiming)

public static final double aprilTagWidth = Units.inchesToMeters(6.50);

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef"),
    NONE("2025-none");

    AprilTagLayoutType(String name) {
      if (Constants.currentMode == Mode.SIM) {
        try {
          layout =
              new AprilTagFieldLayout(
                  Path.of(
                      "src",
                      "main",
                      "deploy",
                      "apriltags",
                      fieldType.getJsonFolder(),
                      "2025-official.json"));
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      } else {
        try {
          layout =
              new AprilTagFieldLayout(
                  Path.of(
                      Filesystem.getDeployDirectory().getPath(),
                      "apriltags",
                      fieldType.getJsonFolder(),
                      name + ".json"));
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }
	private VisionConstants() {
	}
}
