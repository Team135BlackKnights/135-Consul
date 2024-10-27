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
import frc.robot.utils.drive.DriveConstants;

public class VisionConstants {
	// Field layout, fed to the PV cameras in order to work properly
	public static boolean debug = true;
	public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField
			.loadAprilTagLayoutField();

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

	// If the change in odometry is below this distance, do not adjust april tag
	// trusts.
	public final static double maxStaleReadingXMeters = Units.inchesToMeters(4),
			maxStaleReadingYMeters = Units.inchesToMeters(4),
			maxStaleReadingRotation = Units.degreesToRadians(.02);

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
	// Pitches of camera, in DEGREES, positive means UPWARD angle
	public static int
	// Camera resolution, and FPS
	camResWidth = 600, camResHeight = 800, camFPS = 60;
	// Putting all the values together (Creating rotation3ds from the rotation 2ds
	// and putting them together with the translation2ds)
	public static Translation3d frontPos = frontCamTranslation3d,
			rightPos = rightCamTranslation3d, leftPos = leftCamTranslation3d,
			backPos = backCamTranslation3d;
	public static Rotation3d frontRot = new Rotation3d(
			Math.toDegrees(frontCamRoll.get()),
			Math.toRadians(frontCamPitch.get()), 
			Math.toRadians(frontCamYaw.get())),
		rightRot = new Rotation3d(
			Math.toDegrees(rightCamRoll.get()),
			Math.toRadians(rightCamPitch.get()), 
			Math.toRadians(rightCamYaw.get())),
		leftRot = new Rotation3d(			
			Math.toDegrees(leftCamRoll.get()),
			Math.toRadians(leftCamPitch.get()), 
			Math.toRadians(leftCamYaw.get())),
		backRot = new Rotation3d(
			Math.toDegrees(backCamRoll.get()),
			Math.toRadians(backCamPitch.get()), 
			Math.toRadians(backCamYaw.get()));
	//Transforms, used in the camera declarations
	public static Transform3d robotToFront = new Transform3d(frontPos, frontRot),
			robotToRight = new Transform3d(rightPos, rightRot),
			robotToLeft = new Transform3d(leftPos, leftRot),
			robotToBack = new Transform3d(backPos, backRot);
	// Put in an array for easier iterating
	public static Transform3d[] camTranslations = new Transform3d[] {
			robotToFront, robotToLeft, robotToRight, robotToBack
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
	public static final double kMaxVisionCorrection = Units.inchesToMeters(5); // Jump from fused pose
	public static final double kMaxVisionCorrectionSkid = Units
			.inchesToMeters(15); // Jump from fused pose
	public static final double offsetMaxDistance = 4; // in meters
	public static final double std_dev_multiplier = 1;
	public static final double kMaxRotationCorrection = Units
			.degreesToRadians(10);
	public static final double kMaxRotationCorrectionSkid = Units
			.degreesToRadians(20);
	public static final double lowestDistErrorStdDev = 0; // at .125, (5,.3125) (10,.625), at .25 (5,.625) (10,1.125)
	public static final double avgDistErrorStdDev = .025; // at .2, (10, 1) (20, 2), at .4 (10, 2) (20, 4) SET : .025
	public static final double poseAmbiguityErrorStdDev = .25; // at 4, (.15, .3) (.5, 1), at 8 (.15, .6) (.5, 2) SET :
																// .5
	public static final double weighAverageErrorStdDev = .05; // at .25, (.9, .1389) (.5, .25), at .5 (.9, .2777) (.5,
																// .5) SET : .05
	public static final double numTagsMultiplier = 1;
	public static final double kMaxPoseAmbiguity = .6;
	public static final double kMaxPoseAmbiguitySkid = .8;

	public static class FieldConstants {
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		public static double[] aprilTagOffsets = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1
		};
	}
}
