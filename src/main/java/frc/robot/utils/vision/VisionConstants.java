package frc.robot.utils.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.GeomUtil.ApproachDirection;
public class VisionConstants {
	// Field layout, fed to the PV cameras in order to work properly
	public static boolean debug = true;
	public static final AprilTagFields kAprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark;
	//THIS NEEDS TO BE CHANGED TO WELDED FOR WORLDS 
	public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(kAprilTagFieldLayout);
	public static final long WELDED_FIELD_DATE = 1743897600; // Date in human readable format: 2025-06-05
	public static final long ANDYMARK_FIELD_DATE = 1744713633; // Date in human readable format: 2025-06-14
	// This needs to be changed year after year
	public enum AITargets {
		kGamePieceOne(0.0),
		kGamePieceTwo(1.0),
		kRobot(2.0);

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
			"Vision/IntakeCloseEnoughToConsiderMissingDistance", Units.feetToMeters(4), TuningConstants.isTuningVision);
	public static LoggableTunedNumber limelightCloseEnoughToConsiderMissingAngle = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingAngle", Units.degreesToRadians(3),TuningConstants.isTuningVision);
	public static LoggableTunedNumber limelightCloseEnoughToConsiderMissingTimeout = new LoggableTunedNumber(
			"Vision/IntakeCloseEnoughToConsiderMissingTimeout", 1,TuningConstants.isTuningVision);
	public static class Controls {
		public static JoystickButton autoIntake = new JoystickButton(
				RobotContainer.driveController, 8); // start
	}
	// We used 2 cameras for our 2024 year, adjust these accordingly by removing
	// camera names (there are two extra cameras here)
	// Camera names, from photonVision web interface

	// This is a goofy declaration but we use case statements later on and this is
	// the only form of input they accept
	public final static String FCamName = "F_Camera",
			ExtraFrontCamName = "Extra_Front_Camera",
			BCamName = "B_Camera", RCamName = "R_Camera";
	// Check WPILIB Coordinate System
	// Translations should be in inches, Rotations should be in degrees
	public final static double maxVelocity = 0.05, maxAngularVelocity = .05; //.5m/s and .1rad/s
	// To figure out what these should be, look at the WPILIB Coordinate System
	public final static Translation3d FCamTranslation3d = new Translation3d(0.293, 0.337, 0.2124
			),
			ExtraFrontCamTranslation3d = new Translation3d(0.293, -0.337, 0.2124
			),
			BCamTranslation3d = new Translation3d(-0.293, 0.337, 0.2124
					),
			RCamTranslation3d = new Translation3d(-0.2943098, -0.2943098, 0.2124
					);
	public final static Rotation3d FRot = new Rotation3d(
			Math.toRadians(0),
			Math.toRadians(-28.125),
			Math.toRadians(-30)), //33.2?
			ExtraFrontRot = new Rotation3d(
			Math.toRadians(0),
			Math.toRadians(-11.125),
			Math.toRadians(28.3)), //33.2?
			BRot = new Rotation3d(
					Math.toRadians(0),
					Math.toRadians(-28.125),
					Math.toRadians(-150)), //123.2
			RRot = new Rotation3d(
					Math.toRadians(0),
					Math.toRadians(-28.125),
					Math.toRadians(-60));
	// Transforms, used in the camera declarations
	public final static Transform3d robotToF = new Transform3d(FCamTranslation3d, FRot),
			robotToExtraFront = new Transform3d(ExtraFrontCamTranslation3d,ExtraFrontRot),
			robotToB = new Transform3d(BCamTranslation3d, BRot),
			robotToR = new Transform3d(RCamTranslation3d, RRot);
	//Transforms in a list, for easy iteration
	public final static Transform3d[] robotToCameraTransforms = new Transform3d[] {
		robotToExtraFront,robotToF};
	 // Basic filtering thresholds
	 public static double maxAmbiguity = 0.3;
	 public static double maxZError = 0.25;
	 public static double maxYawError = 15;
	 // Standard deviation baselines, for 1 meter distance and 1 tag
	 // (Adjusted automatically based on distance and # of tags)
	 public static double linearStdDevBaseline = 0.005; // Meters
	 public static double angularStdDevBaseline = 0.04; // Radians
   
	 // Standard deviation multipliers for each camera
	 // (Adjust to trust some cameras more than others)
	 public static double[] cameraStdDevFactors =
		 new double[] {
		   1.0, // Camera 0
		   1,
		   1.0, // Camera 1
		   1.0		 };   
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
		public static double[] aprilTagOffsets = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	}
}
