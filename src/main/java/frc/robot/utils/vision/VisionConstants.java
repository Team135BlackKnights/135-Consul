package frc.robot.utils.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.GeomUtil.ApproachDirection;

public class VisionConstants {
	// Field layout, fed to the PV cameras in order to work properly
	public static boolean debug = true;
	public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// This needs to be changed year after year
	public enum AITargets {
		kGamePiece(0.0),
		kRobot(1.0);

		private final double value;

		AITargets(double value) {
			this.value = value;
		}

		public double getValue() {
			return value;
		}
	}
	//Command specific constants
	//Aim To Pose
	public static final ApproachDirection aimToPoseApproachDirection = ApproachDirection.FRONT;
	//Drive And Aim At Pose
	public static final ApproachDirection driveAndAimAtPoseApproachDirection = ApproachDirection.FRONT;
	//Drive To AI Target
	public static final ApproachDirection driveToAITargetApproachDirection = ApproachDirection.FRONT;
	public static LoggableTunedNumber limelightCloseEnoughToConsiderMissingDistance = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingDistance", Units.feetToMeters(4));
	public static LoggableTunedNumber limelightCloseEnoughToConsiderMissingAngle = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingAngle", Units.degreesToRadians(3));
	public static LoggableTunedNumber limelightCloseEnoughToConsiderMissingTimeout = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingTimeout", 1);
	// If the change in odometry is below this distance, do not adjust april tag
	// trusts.
	public final static double maxStaleReadingXMeters = Units.inchesToMeters(4),
			maxStaleReadingYMeters = Units.inchesToMeters(4),
			maxStaleReadingRotation = Units.degreesToRadians(2);

	public static class Controls {
		public static JoystickButton autoIntake = new JoystickButton(
				RobotContainer.driveController, 1); // a
	}
	// We used 2 cameras for our 2024 year, adjust these accordingly by removing
	// camera names (there are two extra cameras here)
	// Camera names, from photonVision web interface

	// This is a goofy declaration but we use case statements later on and this is
	// the only form of input they accept
	public final static String FLCamName = "FL_Camera",
			BRCamName = "BR_Camera", BLCamName = "BL_Camera",
			FRCamName = "FR_Camera";
	// Check WPILIB Coordinate System
	// Translations should be in inches, Rotations should be in degrees
	public static LoggableTunedNumber

	FRCamTranslationX = new LoggableTunedNumber("Vision/FRCamX", 12.626),
			FRCamTranslationY = new LoggableTunedNumber("Vision/FRCamY", -11.001),
			FRCamTranslationZ = new LoggableTunedNumber("Vision/FRCamZ", 8.364),
			FRCamRoll = new LoggableTunedNumber("Vision/FRCamRoll", 0),
			FRCamYaw = new LoggableTunedNumber("Vision/FRCamYaw", -56.8),
			FRCamPitch = new LoggableTunedNumber("Vision/FRCamPitch", -28.125),

			FLCamTranslationX = new LoggableTunedNumber("Vision/FLCamX", 12.626),
			FLCamTranslationY = new LoggableTunedNumber("Vision/FLCamY", 11.001),
			FLCamTranslationZ = new LoggableTunedNumber("Vision/FLCamZ", 8.364),
			FLCamRoll = new LoggableTunedNumber("Vision/FLCamRoll", 0),
			FLCamYaw = new LoggableTunedNumber("Vision/FLCamYaw", 33.2),
			FLCamPitch = new LoggableTunedNumber("Vision/FLCamPitch", -28.125),

			BRCamTranslationX = new LoggableTunedNumber("Vision/BRCamX", -12.626),
			BRCamTranslationY = new LoggableTunedNumber("Vision/BRCamY", -11.001),
			BRCamTranslationZ = new LoggableTunedNumber("Vision/BRCamZ", 8.364),
			BRCamRoll = new LoggableTunedNumber("Vision/BRCamRoll", 0),
			BRCamYaw = new LoggableTunedNumber("Vision/BRCamYaw", -146.8),
			BRCamPitch = new LoggableTunedNumber("Vision/BRCamPitch", -28.125),

			BLCamTranslationX = new LoggableTunedNumber("Vision/BLCamX", -12.626),
			BLCamTranslationY = new LoggableTunedNumber("Vision/BLCamY", 11.001),
			BLCamTranslationZ = new LoggableTunedNumber("Vision/BLCamZ", 8.364),
			BLCamRoll = new LoggableTunedNumber("Vision/BLCamRoll", 0),
			BLCamYaw = new LoggableTunedNumber("Vision/BLCamYaw", 123.2),
			BLCamPitch = new LoggableTunedNumber("Vision/BLCamPitch", -28.125);


	// To figure out what these should be, look at the WPILIB Coordinate System
	public static Translation3d FLCamTranslation3d = new Translation3d(
			Units.inchesToMeters(FLCamTranslationX.get()),
			Units.inchesToMeters(FLCamTranslationY.get()),
			Units.inchesToMeters(FLCamTranslationZ.get())),
			FRCamTranslation3d = new Translation3d(
					Units.inchesToMeters(FRCamTranslationX.get()),
					Units.inchesToMeters(FRCamTranslationY.get()),
					Units.inchesToMeters(FRCamTranslationZ.get())),
			BLCamTranslation3d = new Translation3d(
					Units.inchesToMeters(BLCamTranslationX.get()),
					Units.inchesToMeters(BLCamTranslationY.get()),
					Units.inchesToMeters(BLCamTranslationZ.get())),
			BRCamTranslation3d = new Translation3d(
					Units.inchesToMeters(BRCamTranslationX.get()),
					Units.inchesToMeters(BRCamTranslationY.get()),
					Units.inchesToMeters(BRCamTranslationZ.get()));
	public static Rotation3d FLRot = new Rotation3d(
			Math.toRadians(FLCamRoll.get()),
			Math.toRadians(FLCamPitch.get()),
			Math.toRadians(FLCamYaw.get())),
			FRRot = new Rotation3d(
					Math.toRadians(FRCamRoll.get()),
					Math.toRadians(FRCamPitch.get()),
					Math.toRadians(FRCamYaw.get())),
			BLRot = new Rotation3d(
					Math.toRadians(BLCamRoll.get()),
					Math.toRadians(BLCamPitch.get()),
					Math.toRadians(BLCamYaw.get())),
			BRRot = new Rotation3d(
					Math.toRadians(BRCamRoll.get()),
					Math.toRadians(BRCamPitch.get()),
					Math.toRadians(BRCamYaw.get()));
	// Transforms, used in the camera declarations
	public static Transform3d robotToFL = new Transform3d(FLCamTranslation3d, FLRot),
			robotToFR = new Transform3d(FRCamTranslation3d, FRRot),
			robotToBL = new Transform3d(BLCamTranslation3d, BLRot),
			robotToBR = new Transform3d(BRCamTranslation3d, BRRot);
	//Transforms in a list, for easy iteration
	public static Transform3d[] robotToCameraTransforms = new Transform3d[] {
			robotToFL, robotToFR, robotToBL, robotToBR};
	 // Basic filtering thresholds
	 public static double maxAmbiguity = 0.3;
	 public static double maxZError = 0.75;
	 public static double maxYawError = 5;
	 // Standard deviation baselines, for 1 meter distance and 1 tag
	 // (Adjusted automatically based on distance and # of tags)
	 public static double linearStdDevBaseline = 0.005; // Meters
	 public static double angularStdDevBaseline = 0.04; // Radians
   
	 // Standard deviation multipliers for each camera
	 // (Adjust to trust some cameras more than others)
	 public static double[] cameraStdDevFactors =
		 new double[] {
		   1.0, // Camera 0
		   1.0, // Camera 1
		   1.0,
		   1.0
		 };   
	// Used for distance calculations for AI stuff
	// Offset of your limelight (0 being perpendicular, negative meaning camera lens
	// down)
	public static double limeLightAngleOffsetDegrees = -40,
			// Height from floor to limelight
			limelightLensHeightoffFloorInches = 22.5;
	// For limelightHelpers
	public static String limelightName = "limelight-swerve";
	// For use in drivetoAITarget (PLACEHOLDER VALUE)
	public static double DriveToAITargetKp = .3, DriveToAIMaxAutoTime = 2;
	public static class FieldConstants {
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		public static double[] aprilTagOffsets = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1
		};
	}
}
