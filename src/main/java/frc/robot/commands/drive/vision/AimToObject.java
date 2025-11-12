package frc.robot.commands.drive.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.Swerve.TxTyPoseRecord;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.vision.VisionConstants;

public class AimToObject extends Command {
	private final DrivetrainS drive;
	private final String tagId;
	private final double desiredDistanceMeters;

	private LoggableTunedNumber kPTx = new LoggableTunedNumber("AimToApriltagTx/kP", 4, TuningConstants.isTuningMacros);
	private LoggableTunedNumber kDTx = new LoggableTunedNumber("AimToApriltagTx/kD", 0.6, TuningConstants.isTuningMacros);
	private LoggableTunedNumber kPDistance = new LoggableTunedNumber("AimToApriltagTx/kPDistance", 3.5, TuningConstants.isTuningMacros);
	private LoggableTunedNumber maxSpeed = new LoggableTunedNumber("AimToApriltagTx/maxSpeedMetersPerSec", 4.5, TuningConstants.isTuningMacros);
	private LoggableTunedNumber maxRotation = new LoggableTunedNumber("AimToApriltagTx/MaxRotationRadPerSec", 15, TuningConstants.isTuningMacros);

	// tolerances
	private LoggableTunedNumber txTolerance = new LoggableTunedNumber("AimToApriltagTx/txToleranceRad", .05, TuningConstants.isTuningMacros);
	private LoggableTunedNumber distanceTolerance = new LoggableTunedNumber("AimToApriltagTx/distanceToleranceMeters", Units.inchesToMeters(3), TuningConstants.isTuningMacros);
	private LoggableTunedNumber staleTime = new LoggableTunedNumber("AimToApriltagTx/staleTime",.75,TuningConstants.isTuningMacros);
	private double prevTxRadians = 0.0;
	private double latestTxRadians = 0.0;
	private double latestDistanceMeters = 0.0;
	private boolean hasValidObservation = false;
    private boolean isFinished = false;
    private int camId = 0;

	public AimToObject(DrivetrainS drive, String tagId, double desiredDistanceMeters) {
		this.drive = drive;
		this.tagId = tagId;
		this.desiredDistanceMeters = desiredDistanceMeters;
	}

	@Override
	public void initialize() {
		RobotContainer.currentPath = "AIMTOOBJECT_" + tagId;
		prevTxRadians = 0.0;
        isFinished = false;
	}

	@Override
	public void execute() {
		hasValidObservation = false;
		Optional<TxTyPoseRecord> txTyData = ((Swerve) drive).getTxPoseRecord(tagId);
		if (txTyData.isPresent()) {
			var data = txTyData.get();
			if (Timer.getTimestamp() - data.timestamp() >= staleTime.get() || (!tagId.contains("A") && data.pose().getZ() > VisionConstants.maxObjZError)) {
				hasValidObservation = false;
			}else{
				hasValidObservation = true;
				latestTxRadians = -data.tx();
				latestDistanceMeters = data.distance();
				camId = data.camIndex();
			}
		}else{
			hasValidObservation = false;
		}

		if (!hasValidObservation) {
			drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
			return;
		}

		double angularCommand = 0;
		double forwardCommand = 0;

		double dTx = latestTxRadians - prevTxRadians;

		// only compute angular if outside tolerance
		if (Math.abs(latestTxRadians) > txTolerance.get()) {
			angularCommand = kPTx.get() * latestTxRadians + kDTx.get() * dTx;
			angularCommand = Math.max(-maxRotation.get(), Math.min(maxRotation.get(), angularCommand));
		}

		// only compute forward if outside tolerance
		double distanceError = latestDistanceMeters - desiredDistanceMeters;
		if (Math.abs(distanceError) > distanceTolerance.get()) {
			forwardCommand = kPDistance.get() * distanceError;
			forwardCommand = Math.max(-maxSpeed.get(), Math.min(maxSpeed.get(), forwardCommand));
		}

		// Get current robot pose
		Pose2d robotPose = drive.getPose();
		
		// Get camera-to-robot transform (this is the offset of camera from robot origin)
		Transform2d robotToCamera = GeomUtil.poseToTransform(VisionConstants.cameras[camId].getPose().get().toPose2d());
		
		// Calculate camera pose in field coordinates
		Pose2d cameraPose = robotPose.plus(robotToCamera);
		
		// The AprilTag is at distance `latestDistanceMeters` and angle `tx` from the camera
		// Camera's heading + tx gives us the field-relative direction to the AprilTag
		Rotation2d directionToTag = cameraPose.getRotation().plus(new Rotation2d(latestTxRadians));
		
		// The direction vector pointing from camera toward the OBJ
		Translation2d directionVector = new Translation2d(
			Math.cos(directionToTag.getRadians()),
			Math.sin(directionToTag.getRadians())
		);
		
		// Scale by forwardCommand (positive means move closer, negative means move away)
		Translation2d driveVelocity = directionVector.times(forwardCommand);
		
		drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
				driveVelocity.getX(), 
				driveVelocity.getY(),
				angularCommand,
				robotPose.getRotation()));

		prevTxRadians = latestTxRadians;

		Logger.recordOutput("AimToObject/tx", latestTxRadians);
		Logger.recordOutput("AimToObject/distance", latestDistanceMeters);
		Logger.recordOutput("AimToObject/forwardCommand", forwardCommand);
		Logger.recordOutput("AimToObject/angularCommand", angularCommand);
		Logger.recordOutput("AimToObject/directionToTag", directionToTag.getDegrees());
	}

	@Override
	public void end(boolean interrupted) {
		drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
		RobotContainer.currentPath = "";
        isFinished = true;
	}

	@Override
	public boolean isFinished() {
		return isFinished;
	}
}