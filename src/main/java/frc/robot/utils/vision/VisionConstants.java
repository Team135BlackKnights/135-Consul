package frc.robot.utils.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;

public class VisionConstants {
	//Field layout, fed to the PV cameras in order to work properly
	public static boolean debug = true;
	public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField
			.loadAprilTagLayoutField();

	public static class Controls {
		public static JoystickButton autoIntake = new JoystickButton(
				RobotContainer.driveController, 1); //a
	}

	//We put the cameras into an enum to make iteration easier
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
	//We used 2 cameras for our 2024 year, adjust these accordingly by removing camera names (there are two extra cameras here)
	//Camera names, from photonVision web interface

	//This is a goofy declaration but we use case statements later on and this is the only form of input they accept
	public final static String frontCamName = "Front_Camera",
			backCamName = "Back_Camera", leftCamName = "Left_Camera",
			rightCamName = "Right_Camera";
	//offset of each cam from robot center, in meters
	//To figure out what these should be, look at the WPILIB Coordinate System
	public static Translation3d frontCamTranslation3d = new Translation3d(
			Units.inchesToMeters(15), 0, Units.inchesToMeters(19.75)),
			rightCamTranslation3d = new Translation3d(Units.inchesToMeters(12.75),
					Units.inchesToMeters(DriveConstants.kChassisLength / 2),
					Units.inchesToMeters(19.75)),
			leftCamTranslation3d = new Translation3d(Units.inchesToMeters(12.75),
					Units.inchesToMeters(-DriveConstants.kChassisLength / 2),
					Units.inchesToMeters(19.75)),
			backCamTranslation3d = new Translation3d(Units.inchesToMeters(12.75),
					Units.inchesToMeters(0), Units.inchesToMeters(25));
	//Pitches of camera, in DEGREES, positive means UPWARD angle
	public static int
	//Figure these out on your robot
	frontCamPitch = -21, rightCamPitch = -21, leftCamPitch = -15,
			backCamPitch = -10,
			//Camera resolution, and FPS
			camResWidth = 600, camResHeight = 800, camFPS = 60;
	public static int[] camPitches = new int[] {
			//Convert pitches into an array through iteration
			frontCamPitch, leftCamPitch, rightCamPitch, backCamPitch
	};
	//Putting all the values together (Creating rotation3ds from the rotation 2ds and putting them together with the translation2ds)
	public static Translation3d frontPos = frontCamTranslation3d,
			rightPos = rightCamTranslation3d, leftPos = leftCamTranslation3d,
			backPos = backCamTranslation3d;
	public static Rotation3d frontRot = new Rotation3d(0,
			Math.toRadians(frontCamPitch), Math.toRadians(0)),
			rightRot = new Rotation3d(0, Math.toRadians(rightCamPitch),
					Math.toRadians(-90)),
			leftRot = new Rotation3d(0, Math.toRadians(leftCamPitch),
					Math.toRadians(90)),
			backRot = new Rotation3d(0, Math.toRadians(backCamPitch),
					Math.toRadians(180));
	//Transforms, used in the camera declarations
	public static Transform3d robotToFront = new Transform3d(frontPos, frontRot),
			robotToRight = new Transform3d(rightPos, rightRot),
			robotToLeft = new Transform3d(leftPos, leftRot),
			robotToBack = new Transform3d(backPos, backRot);
	//Put in an array for easier iterating
	public static Transform3d[] camTranslations = new Transform3d[] {
			robotToFront, robotToLeft, robotToRight, robotToBack
	};
	//Used for distance calculations for AI stuff
	//Offset of your limelight (0 being perpendicular, negative meaning camera lens down)
	public static double limeLightAngleOffsetDegrees = -40,
			//Height from floor to limelight
			limelightLensHeightoffFloorInches = 22.5;
	//For limelightHelpers
	public static String limelightName = "limelight-swerve";
	//For use in drivetoAITarget (PLACEHOLDER VALUE)
	public static double DriveToAITargetKp = .3, DriveToAIMaxAutoTime = 2;
	public static final double kMaxVisionCorrection = Units.inchesToMeters(5); // Jump from fused pose
	public static final double kMaxVisionCorrectionSkid = Units
			.inchesToMeters(15); // Jump from fused pose
	public static final double offsetMaxDistance = 4; //in meters
	public static final double std_dev_multiplier = 1;
	public static final double kMaxRotationCorrection = Units
			.degreesToRadians(10);
	public static final double kMaxRotationCorrectionSkid = Units
			.degreesToRadians(20);
    public static final double lowestDistErrorStdDev = 0; //at .125, (5,.3125) (10,.625), at .25 (5,.625) (10,1.125)
    public static final double avgDistErrorStdDev = .025; // at .2, (10, 1) (20, 2), at .4 (10, 2) (20, 4)   SET : .025
    public static final double poseAmbiguityErrorStdDev = .25; //at 4, (.15, .3) (.5, 1), at 8 (.15, .6) (.5, 2) SET : .5
    public static final double weighAverageErrorStdDev = .05; // at .25, (.9, .1389) (.5, .25), at .5 (.9, .2777) (.5, .5) SET : .05
	 public static final double numTagsMultiplier = 1;
	public static final double kMaxPoseAmbiguity = .6;
    public static final double kMaxPoseAmbiguitySkid = .8;
	public static class FieldConstants {
		public static final double kFieldLength = Units.inchesToMeters(651.223);
		public static final double kFieldWidth = Units.inchesToMeters(323.277);
		public static final double kFieldBorderMargin = 0.5;
		public static final double kFieldTagMinTrust = .8;
		public static double[] aprilTagOffsets = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				1, 1, 1, 1, 1, 1, 1
		};
	}
}
