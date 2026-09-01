package frc.robot.utils.vision;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Filesystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import io.avaje.jsonb.Jsonb;

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

/**
 * Robot-side constants for vision tuning and Southmoon camera configuration.
 *
 * <p>Anything published by the Southmoon VisionIO implementation comes from
 * here, so changes to camera IDs, exposure, resolution, tag size, HSV
 * thresholds, or target class IDs affect the Mac vision process on the next
 * robot restart/deploy.
 */
public class VisionConstants {
	public static final FieldType fieldType = FieldType.ANDYMARK;

	public enum AITargets {
		// Keep this enum in the exact same order as the object model class IDs.
		// Adding a model class requires updating this enum and any robot code that
		// asks for a specific classId.
		FUEL

	}
	// HSV thresholds used by the simple color object pipeline. These are sent to
	// Southmoon over NT4; the trained .mlmodel object detector has its own model
	// weights and confidence threshold.
	public static final long[] objLowerHSV = {0,0,100};
	public static final long[] objUpperHSV = {180,45,210};
	// Command specific constants // Aim To Pose
	public static final ApproachDirection aimToPoseApproachDirection = ApproachDirection.FRONT, // Drive And Aim At Pose public static final ApproachDirection
			driveAndAimAtPoseApproachDirection = ApproachDirection.FRONT; // Drive To AI
	public static final ApproachDirection driveToAITargetApproachDirection = ApproachDirection.FRONT;

	public static class FieldConstants {
		// Do not trust tags too close to field borders, and allow individual tags to
		// be down-weighted if they repeatedly produce rejected pose estimates.
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		// Standard deviation multiplier per tag ID, indexed by tagId - 1. 1.0 is
		// normal trust; larger values make measurements involving that tag weaker.
		public static double[] aprilTagOffsets = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
	}

	public static final boolean debug = true;

	// Pose rejection/tuning constants. These are deliberately conservative gates:
	// reject impossible measurements here, then use std devs to express uncertainty
	// for measurements that are merely noisy.
	public static final double ambiguityThreshold = 0.7;
	public static final double objDetectConfidenceThreshold = .6;
	public static final double maxZError = 0.75;
	public static final double maxObjZError = 0.25;
	public static final double maxYawError = 90.0;
	public static final double linearStdDevBaseline = 0.025;
	public static final double angularStdDevBaseline = 0.04;
	public static final LoggableTunedNumber shootingGyroYawTrustScale = new LoggableTunedNumber(
			"Vision/ShootingGyroYawTrustScale", 0.7, TuningConstants.isTuningVision);

	// Temporary live offsets for tuning measured camera poses from the dashboard.
	// After tuning, bake the final values into the camera pose below.
	public static final LoggableTunedNumber offsetPoseX = new LoggableTunedNumber("Cams/X",.0,true); //was .05
	public static final LoggableTunedNumber offsetPoseY = new LoggableTunedNumber("Cams/Y",.0,true); //was .035
	// Limelight remains a separate object/intake pipeline. Southmoon cameras do
	// AprilTags and custom object detections; this name must match the Limelight
	// NetworkTables device name.
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

	// Used by Vision.staleReading to avoid repeatedly learning tag trust from the
	// same parked pose.
	public static final double maxStaleReadingXMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingYMeters = Units.inchesToMeters(4);
	public static final double maxStaleReadingRotation = Units.degreesToRadians(2);

	/**
	 * ["0x01210000 / 3", "0x01230000 / 4", "0x02211000 / 5", "0x02213000 / 6"]
'SPCA2630 PC Camera:usb_05c8_0a00_002_003', 'SPCA2630 PC Camera:usb_05c8_0a00_000_001'
	 */
	// Camera order must match the VisionIO construction order and CameraID enum
	// expectations. Poses are robot-to-camera transforms in WPILib coordinates:
	// +X forward, +Y left, +Z up, meters/radians. Intrinsics are not stored here;
	// Southmoon loads camera_matrix and distortion_coefficients from calibration.json.
	public static final CameraConfig[] cameras = new CameraConfig[] {
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									Units.inchesToMeters(0), //centered on robot
									Units.inchesToMeters(0),
									Units.inchesToMeters(0), //it changes, we don't know
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(25),
											Math.toRadians(0)))) //we don't know, but it faces forward so we assume no yaw/pitch
										.id("Arducam OV9782 USB Camera:usb_0c45_6366_UC852")
					.location("0x03130000 / 3")
					.width(1280)//1600
					.height(800)//1304
					.exposure(35)
					.saturation(100)
					.hue(0)
					.whiteBalance(4500)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
			CameraConfig.builder()
					.pose(
							() -> new Pose3d(
								-.317,
								-.263,
								.205,
								new Rotation3d(
										Math.toRadians(0.0),
										Math.toRadians(-25),
										Math.toRadians(160))))
										.id("SPCA2630 PC Camera:usb_05c8_0a00_002_003")
					.location("0x02210000 / 3")
										.width(1600)
					.height(1304)
					.exposure(20)
					.saturation(0)
					.hue(0)
					.whiteBalance(4000)
					.autoWhiteBalance(0)
					.autoExposure(0)
					.gain(0)
					.build(),
					CameraConfig.builder()
					.pose(
							() -> new Pose3d(
									-.317 + offsetPoseX.get(),
									.263 + offsetPoseY.get(),
									.205,
									new Rotation3d(
											Math.toRadians(0.0),
											Math.toRadians(-25),
											Math.toRadians(-160))))
											.id("SPCA2630 PC Camera:usb_05c8_0a00_000_004")
										.location("0x00220000 / 4")
					.width(1600)
					.height(1304)
					.exposure(15)
					.saturation(0)
					.hue(0)
					.whiteBalance(4000)
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
		// id/location identify the physical USB camera on the Mac. The remaining
		// fields are capture settings published to Southmoon over NT4.
		private Supplier<Pose3d> pose;
		private String id;
		private String location;
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

// Physical AprilTag/ArUco marker side length used by solvePnP on the coprocessor.
public static final double aprilTagWidth = Units.inchesToMeters(6.50);
public static final boolean bumperDetection = false;
@RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
	OFFSEASON("offseason"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
	HOME("2026-home"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              // Sim uses the source-tree deploy folder; the real robot reads from
              // the roboRIO deploy directory. getLayoutString() sends this same
              // JSON to Southmoon so both sides use the same tag coordinates.
              Path p =
                  Constants.currentMode == Mode.SIM
                      ? Path.of(
                          "src",
                          "main",
                          "deploy",
                          "apriltags",
                          fieldType.getJsonFolder(),
                          "2026-sim" + ".json")
                      : Path.of(
                          Filesystem.getDeployDirectory().getPath(),
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = Jsonb.instance().toJson(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}
