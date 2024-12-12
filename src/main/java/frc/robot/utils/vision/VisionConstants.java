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
	public static final ApproachDirection aimToPoseApproachDirection = ApproachDirection.BACK;
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

	// We put the cameras into an enum to make iteration easier
	public static enum PVCameras {
		Front_Camera, Left_Camera, Right_Camera, Back_Camera;

		public static PVCameras getCameraByIndex(int index) {
			if (index < 0 || index >= PVCameras.values().length) {
				throw new IndexOutOfBoundsException(
						"Index out of bounds for PVCameras enum.");
			}
			return PVCameras.values()[index];
		}
	}
	// We used 2 cameras for our 2024 year, adjust these accordingly by removing
	// camera names (there are two extra cameras here)
	// Camera names, from photonVision web interface

	// This is a goofy declaration but we use case statements later on and this is
	// the only form of input they accept
	public final static String frontCamName = "Front_Camera",
			backCamName = "Back_Camera", leftCamName = "Left_Camera",
			rightCamName = "Right_Camera";
	// Check WPILIB Coordinate System
	// Translations should be in inches, Rotations should be in degrees
	public static LoggableTunedNumber

	frontCamTranslationX = new LoggableTunedNumber("Vision/FrontCamX", 12.75),
			frontCamTranslationY = new LoggableTunedNumber("Vision/FrontCamY", 12.75),
			frontCamTranslationZ = new LoggableTunedNumber("Vision/FrontCamZ", 19.75),
			frontCamRoll = new LoggableTunedNumber("Vision/FrontCamRoll", 180),
			frontCamYaw = new LoggableTunedNumber("Vision/FrontCamYaw", 95.8),
			frontCamPitch = new LoggableTunedNumber("Vision/FrontCamPitch", -32),

			rightCamTranslationX = new LoggableTunedNumber("Vision/RightCamX", 12.75),
			rightCamTranslationY = new LoggableTunedNumber("Vision/RightCamY", 12.75),
			rightCamTranslationZ = new LoggableTunedNumber("Vision/RightCamZ", 19.75),
			rightCamRoll = new LoggableTunedNumber("Vision/RightCamRoll", 180),
			rightCamYaw = new LoggableTunedNumber("Vision/RightCamYaw", 95.8),
			rightCamPitch = new LoggableTunedNumber("Vision/RightCamPitch", -32),

			leftCamTranslationX = new LoggableTunedNumber("Vision/LeftCamX", 11.125),
			leftCamTranslationY = new LoggableTunedNumber("Vision/LeftCamY", -12.125),
			leftCamTranslationZ = new LoggableTunedNumber("Vision/LeftCamZ", 14.125),
			leftCamRoll = new LoggableTunedNumber("Vision/LeftCamRoll", 174),
			leftCamYaw = new LoggableTunedNumber("Vision/LeftCamYaw", -100),
			leftCamPitch = new LoggableTunedNumber("Vision/LeftCamPitch", -5),

			backCamTranslationX = new LoggableTunedNumber("Vision/BackCamX", 12.125),
			backCamTranslationY = new LoggableTunedNumber("Vision/BackCamY", 25.5),
			backCamTranslationZ = new LoggableTunedNumber("Vision/BackCamZ", 21.25),
			backCamPitch = new LoggableTunedNumber("Vision/BackCamPitch", 38),
			backCamRoll = new LoggableTunedNumber("Vision/BackCamRoll", 180),
			backCamYaw = new LoggableTunedNumber("Vision/BackCamYaw", 183);

	// To figure out what these should be, look at the WPILIB Coordinate System
	public static Translation3d frontCamTranslation3d = new Translation3d(
			Units.inchesToMeters(frontCamTranslationX.get()),
			Units.inchesToMeters(frontCamTranslationY.get()),
			Units.inchesToMeters(frontCamTranslationZ.get())),
			rightCamTranslation3d = new Translation3d(
					Units.inchesToMeters(rightCamTranslationX.get()),
					Units.inchesToMeters(rightCamTranslationY.get()),
					Units.inchesToMeters(rightCamTranslationZ.get())),
			leftCamTranslation3d = new Translation3d(
					Units.inchesToMeters(leftCamTranslationX.get()),
					Units.inchesToMeters(leftCamTranslationY.get()),
					Units.inchesToMeters(leftCamTranslationZ.get())),
			backCamTranslation3d = new Translation3d(
					Units.inchesToMeters(backCamTranslationX.get()),
					Units.inchesToMeters(backCamTranslationY.get()),
					Units.inchesToMeters(backCamTranslationZ.get()));
	public static Rotation3d frontRot = new Rotation3d(
			Math.toRadians(frontCamRoll.get()),
			Math.toRadians(frontCamPitch.get()),
			Math.toRadians(frontCamYaw.get())),
			rightRot = new Rotation3d(
					Math.toRadians(rightCamRoll.get()),
					Math.toRadians(rightCamPitch.get()),
					Math.toRadians(rightCamYaw.get())),
			leftRot = new Rotation3d(
					Math.toRadians(leftCamRoll.get()),
					Math.toRadians(leftCamPitch.get()),
					Math.toRadians(leftCamYaw.get())),
			backRot = new Rotation3d(
					Math.toRadians(backCamRoll.get()),
					Math.toRadians(backCamPitch.get()),
					Math.toRadians(backCamYaw.get()));
	// Transforms, used in the camera declarations
	public static Transform3d robotToFront = new Transform3d(frontCamTranslation3d, frontRot),
			robotToRight = new Transform3d(rightCamTranslation3d, rightRot),
			robotToLeft = new Transform3d(leftCamTranslation3d, leftRot),
			robotToBack = new Transform3d(backCamTranslation3d, backRot);
	 // Basic filtering thresholds
	 public static double maxAmbiguity = 0.3;
	 public static double maxZError = 0.75;
   
	 // Standard deviation baselines, for 1 meter distance and 1 tag
	 // (Adjusted automatically based on distance and # of tags)
	 public static double linearStdDevBaseline = 0.02; // Meters
	 public static double angularStdDevBaseline = 0.06; // Radians
   
	 // Standard deviation multipliers for each camera
	 // (Adjust to trust some cameras more than others)
	 public static double[] cameraStdDevFactors =
		 new double[] {
		   1.0, // Camera 0
		   1.0 // Camera 1
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
