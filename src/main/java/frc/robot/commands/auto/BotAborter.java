package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.Reef;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Simulation.CompetitionFieldSimulation;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.AITargets;

import java.util.List;
import java.util.ArrayList;
import edu.wpi.first.math.Pair;
@SuppressWarnings("unused")
public class BotAborter extends Command {
	final DrivetrainS drive;
	private Pose2d currentPose, previousOpposingBotPose = null;
	private boolean isFinished;
	private Translation2d targetPieceLocation;
	double deltaTime = .02;
	List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
	final double robotWidth = Units.inchesToMeters(30.0) // Width of the opponent robot in inches
			, robotLength = Units.inchesToMeters(30.0); // Length of the opponent robot in inche

	public BotAborter(DrivetrainS drivetrainS) { this.drive = drivetrainS; }

	@Override
	public void initialize() {
		isFinished = false;
		if (Constants.currentMode == Mode.SIM) {
			//If the robot is in sim, target the closest game piece to drive to
			this.targetPieceLocation = CompetitionFieldSimulation
					.getClosestGamePiece(Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated.class, drive.getPose().getTranslation()).getTranslation();
		}
	}

	@Override
	public void execute() {
		double gamePieceTx = 0, gamePieceTy = 0, gamePieceDistance = 0,
				robotTx = 0, robotTy = 0;
		boolean gamePieceTv = false, robotTv = false;
		if (Constants.currentMode == Mode.SIM) {
			//In simulation, get the current pose, and set the degree value to 
			currentPose = drive.getPose();
			targetPieceLocation = CompetitionFieldSimulation
					.getClosestGamePiece(Reefscape2025FieldObjects.ReefscapeCoralOnFieldSimulated.class, drive.getPose().getTranslation()).getTranslation();
			if (targetPieceLocation == null) {
				return;
			}
			double deltaX = targetPieceLocation.getX() - currentPose.getX();
			double deltaY = targetPieceLocation.getY() - currentPose.getY();
			gamePieceTx = Units.radiansToDegrees(Math.atan2(deltaY, deltaX)); // Use atan2 instead of atan
			gamePieceTx -= currentPose.getRotation().getDegrees();
			//gamePieceTx = GeomUtil.closerAngleToZero(Rotation2d.fromDegrees(gamePieceTx));
			double d = currentPose.getTranslation()
					.getDistance(targetPieceLocation);
			double tyRad = Math.PI
					- Units.degreesToRadians(
							VisionConstants.limeLightAngleOffsetDegrees)
					- (Math.PI * 0.5D - Math.atan(d / Units.inchesToMeters(
							VisionConstants.limelightLensHeightoffFloorInches)));
			gamePieceTy = Units.radiansToDegrees(tyRad);
			gamePieceTv = true;
			if (Constants.currentMatchState == FRCMatchState.AUTO) {
				Pose2d opposingBotPose = CompetitionFieldSimulation
						.getClosestRobotPose(currentPose.getTranslation());
				Logger.recordOutput("OpposingRobot/GivenPose", opposingBotPose);
				double robotDeltaX = opposingBotPose.getX() - currentPose.getX();
				double robotDeltaY = opposingBotPose.getY() - currentPose.getY();
				robotTx = Units
						.radiansToDegrees(Math.atan2(robotDeltaY, robotDeltaX)); // Use atan2 instead of atan
				robotTx -= currentPose.getRotation().getDegrees();
				//robotTx = GeomUtil.closerAngleToZero(Rotation2d.fromDegrees(robotTx));
				double robotD = opposingBotPose.getTranslation()
						.getDistance(currentPose.getTranslation());
				double robotTyRad = Math.PI
						- Units.degreesToRadians(
								VisionConstants.limeLightAngleOffsetDegrees)
						- (Math.PI * 0.5D - Math.atan(robotD / Units.inchesToMeters(
								VisionConstants.limelightLensHeightoffFloorInches)));
				robotTy = Units.radiansToDegrees(robotTyRad);
				robotTv = true;
			}
		} else {
			//THESE ARE IN D E G R E E S 
			LimelightHelpers.LimelightTarget_Detector[] results = LimelightHelpers
					.getLatestResults(
							VisionConstants.limelightName).targetingResults.targets_Detector;
			for (LimelightHelpers.LimelightTarget_Detector object : results) {
				if (object.confidence < .4) {
					continue;
				}
				if (object.classID == AITargets.BLUE_BOT.ordinal()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					gamePieceTv = true;
				} else {
					gamePieceTv = false;
					if (object.classID == AITargets.RED_BOT.ordinal()) {
						robotTx = -object.tx;
						robotTy = object.ty;
						robotTv = true;
					} else {
						robotTv = false;
					}
				}
			}
		}
		/*Pose3d estimatedgamePiecePose3d = GeomUtil.calculateFieldRelativePose3d(
				currentPose, gamePieceTx, gamePieceTy,
				Units.inchesToMeters(
						VisionConstants.limelightLensHeightoffFloorInches),
				Units.inchesToMeters(2),
				VisionConstants.limeLightAngleOffsetDegrees);
		Logger.recordOutput("BotAborterGamePiece",
				estimatedgamePiecePose3d);
		gamePieceDistance = GeomUtil.calculateDistanceFromPose3d(currentPose,
				estimatedgamePiecePose3d);
		if (robotTv && gamePieceTv && RobotContainer.currentPath != "INTAKING_DRIVEPOSE") {
			Pose3d estimatedOpposingBotPose3d = GeomUtil
					.calculateFieldRelativePose3d(currentPose, robotTx, robotTy,
							Units.inchesToMeters(
									VisionConstants.limelightLensHeightoffFloorInches),
							Units.inchesToMeters(2),
							VisionConstants.limeLightAngleOffsetDegrees);
			Logger.recordOutput("OpposingRobot/Pose", estimatedOpposingBotPose3d); //assume robot is 30x30
			obstacles.clear();
			Translation2d frontRightCorner = estimatedOpposingBotPose3d.toPose2d()
					.getTranslation()
					.plus(new Translation2d(robotLength / 2, robotWidth / 2));
			Translation2d backLeftCorner = estimatedOpposingBotPose3d.toPose2d()
					.getTranslation()
					.plus(new Translation2d(-robotLength / 2, -robotWidth / 2));
			obstacles.add(new Pair<>(frontRightCorner, backLeftCorner));
			Logger.recordOutput("OpposingRobot/FrontRightCorner", frontRightCorner);
			Logger.recordOutput("OpposingRobot/BackLeftCorner", backLeftCorner);
			Pathfinding.setDynamicObstacles(obstacles,
					drive.getPose().getTranslation());
			double opposinggamePieceDistance = GeomUtil
					.calculateDistanceFromPose3d(estimatedgamePiecePose3d.toPose2d(),
							estimatedOpposingBotPose3d);
			if (previousOpposingBotPose != null) {
				double deltaX = estimatedOpposingBotPose3d.toPose2d().getX()
						- previousOpposingBotPose.getX();
				double deltaY = estimatedOpposingBotPose3d.toPose2d().getY()
						- previousOpposingBotPose.getY();
				double velocityX = deltaX / deltaTime;
				double velocityY = deltaY / deltaTime;
				double opposingRobotSpeedTowardsgamePiece = Math.hypot(velocityX,
						velocityY);
				Twist2d ourVelocity = drive.getFieldVelocity();
				double ourSpeedTowardsgamePiece = Math.hypot(ourVelocity.dx,
						ourVelocity.dy);
				double ourTimeTogamePiece = gamePieceDistance
						/ (ourSpeedTowardsgamePiece + .001); //avoid divide/0 crash
				double opposingRobotTimeTogamePiece = opposinggamePieceDistance
						/ (opposingRobotSpeedTowardsgamePiece + .001);
				Logger.recordOutput("OpposingRobot/VelocityUs", ourSpeedTowardsgamePiece);
				Logger.recordOutput("OpposingRobot/VelocityThem", opposingRobotSpeedTowardsgamePiece);
				if (opposingRobotTimeTogamePiece < ourTimeTogamePiece + .25) {
					isFinished = true;
				}
			}
			previousOpposingBotPose = estimatedOpposingBotPose3d.toPose2d();
		}*/
	}

	@Override
	public void end(boolean interrupted) {
		Pathfinding.setDynamicObstacles(new ArrayList<>(),
				drive.getPose().getTranslation());
		previousOpposingBotPose = null;
	}

	@Override
	public boolean isFinished() { return isFinished; }
}