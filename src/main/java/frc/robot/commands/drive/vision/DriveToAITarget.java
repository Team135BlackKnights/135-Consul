package frc.robot.commands.drive.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.Mode;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.CompetitionFieldUtils.Simulation.CompetitionFieldSimulation;
import frc.robot.utils.drive.DriveConstants;

/**
 * Implementation of this command is a little different. You must have a way to
 * detect a game piece within the robot, or however you END this command. For an
 * example methodology on this, please refer to the GITHUB 135 Crescendo2024 for
 * IntakeS, and read through the ways we detected game pieces (color sensor
 * because the object was a bright color!) Beyond that, the distance value must
 * be changed within Constants, where that value is in INCHES.
 */
public class DriveToAITarget extends Command {
	private final DrivetrainS swerveS;
	private boolean isFinished = false, goingToCenterLine = false,
			loaded = false; //do we have game Piece?
	public static boolean takeOver = false; //to stop driver input
	private static boolean close = false; //when close, stop moving
	private Translation2d targetPieceLocation = null; //no target known unless in SIM
	private Pose2d currentPose;
	private ChassisSpeeds speeds;
	private Command drivingToCenterLine = null;
	Timer timer = new Timer();
	double desiredHeading, currentHeading;

	/*
	 * Prpvide the drive subsystem.
	 */
	public DriveToAITarget(DrivetrainS swerveS) {
		this.swerveS = swerveS;
		addRequirements(swerveS);
	}

	@Override
	public void initialize() {
		if (Constants.currentMode == Mode.SIM) {
			//If the robot is in sim, target the closest game piece to drive to
			this.targetPieceLocation = CompetitionFieldSimulation.getClosestGamePiece(swerveS.getPose().getTranslation());
		}
		isFinished = false;
		LimelightHelpers.setPipelineIndex(VisionConstants.limelightName, 1);
		timer.reset();
		timer.start();
		close = false;
		takeOver = true; //stop user control
		RobotContainer.currentPath = "INTAKING";
	}

	@Override
	public void execute() {
		//determine if game piece loaded
		if (loaded) {
			isFinished = true;
		}
		double gamePieceTx = 0, gamePieceTy = 0, gamePieceDistance = 0;
		boolean gamePieceTv = false;
		if (Constants.currentMode == Mode.SIM) {
			//In simulation, get the current pose, and set the degree value to 
			currentPose = swerveS.getPose();
			double deltaX = targetPieceLocation.getX() - currentPose.getX();
			double deltaY = targetPieceLocation.getY() - currentPose.getY();
			gamePieceTx = Units.radiansToDegrees(Math.atan2(deltaY, deltaX)); // Use atan2 instead of atan
			gamePieceTx -= currentPose.getRotation().getDegrees();
			gamePieceTx = GeomUtil.closerAngleToZero(gamePieceTx);
			double d = currentPose.getTranslation()
					.getDistance(targetPieceLocation);
			double tyRad = Math.PI
					- Units.degreesToRadians(
							VisionConstants.limeLightAngleOffsetDegrees)
					- (Math.PI * 0.5D - Math.atan(d / Units.inchesToMeters(
							VisionConstants.limelightLensHeightoffFloorInches)));
			gamePieceTy = Units.radiansToDegrees(tyRad);
			gamePieceTv = true;
		} else {
			//THESE ARE IN D E G R E E S 
			LimelightHelpers.LimelightTarget_Detector[] results = LimelightHelpers
					.getLatestResults(
							VisionConstants.limelightName).targetingResults.targets_Detector;
			for (LimelightHelpers.LimelightTarget_Detector object : results) {
				if (object.confidence < .4) {
					continue;
				}
				if (object.className == "note") {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					gamePieceTv = true;
				} else {
					gamePieceTv = false;
				}
			}
		}
		Pose3d estimatedgamePiecePose3d = GeomUtil
		.calculateFieldRelativePose3d(currentPose, gamePieceTx,
				gamePieceTy,
				Units.inchesToMeters(
						VisionConstants.limelightLensHeightoffFloorInches),
				Units.inchesToMeters(2),
				VisionConstants.limeLightAngleOffsetDegrees);
Logger.recordOutput("SIMINTAKEgamePiece", estimatedgamePiecePose3d);
gamePieceDistance = GeomUtil.calculateDistanceFromPose3d(currentPose,
		estimatedgamePiecePose3d);
if (VisionConstants.debug) {
	SmartDashboard.putNumber("tx", gamePieceTx);
	SmartDashboard.putNumber("ty", gamePieceTy);
	SmartDashboard.putNumber("DISTANCE", gamePieceDistance);
}
// SmartDashboard.putBoolean("Piece Loaded?", IntakeS.PieceIsLoaded());
if (gamePieceDistance <= Units.inchesToMeters(4.5)) { //less than 4.5 inches away, STOP!
	if (Constants.currentMode == Constants.Mode.SIM) {
		isFinished = true;
	}
	close = true;
}
		if (Constants.currentMatchState == FRCMatchState.AUTO
				&& timer.get() > 1) {
			isFinished = true; //if in auto, and greater than max time, STOP ENTIRE COMMAND
		}
		if (gamePieceTv == false && loaded == false && close == false) {
			// We don't see the target, seek for the target by spinning in place at a safe speed.
			if (Constants.currentMatchState == FRCMatchState.AUTO
					&& !goingToCenterLine) {
				if (Robot.isRed) {
					drivingToCenterLine = new DriveToPose(swerveS,
							new Pose2d(8.27, currentPose.getY(),
									new Rotation2d((Units.degreesToRadians(-90)))));
					drivingToCenterLine.initialize();
					goingToCenterLine = true;
				} else {
					drivingToCenterLine = new DriveToPose(swerveS,
							new Pose2d(8.27, currentPose.getY(),
									new Rotation2d((Units.degreesToRadians(90)))));
					drivingToCenterLine.initialize();
					goingToCenterLine = true;
				}
			} else {
				speeds = new ChassisSpeeds(0, 0,
						0.1 * DriveConstants.kMaxTurningSpeedRadPerSec);
			}
		} else if (loaded == false && close == false) { //We see a target, and we're not close.
			goingToCenterLine = false;
			double speedMapperVal = GeomUtil
					.speedMapper(Units.metersToInches(gamePieceDistance)); //change speed based on distance
			double moveSpeed = DriveConstants.kMaxSpeedMetersPerSecond
					* speedMapperVal;
			double trueError = gamePieceTx; //Add/subtract from this for any tweaking from where camera placed for actual robot error
			double turnSpeed = Units.degreesToRadians(trueError)
					* VisionConstants.DriveToAITargetKp
					* DriveConstants.kMaxTurningSpeedRadPerSec
					* (2 * speedMapperVal);//Make turning relative to distance, and a Kp
			speeds = new ChassisSpeeds(moveSpeed, 0, turnSpeed);
		} else {
			speeds = new ChassisSpeeds(0, 0, 0); //We either have it, or are close enough to start just intaking, so stop.
		}
		if (!goingToCenterLine) {
			swerveS.setChassisSpeeds(speeds);
		} else {
			drivingToCenterLine.execute();
			if (drivingToCenterLine.isFinished()) {
				goingToCenterLine = false;
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		//Return control of the drivetrain
		takeOver = false;
		if (drivingToCenterLine != null) {
			drivingToCenterLine.cancel();
		}
		timer.stop();
		timer.reset();
		speeds = new ChassisSpeeds(0, 0, 0);
		swerveS.setChassisSpeeds(speeds);
		//TODO: Intake the game piece within new physics sim
		/*if (Constants.currentMode == Mode.SIM && close) {
			if (SimGamePiece.currentPieces.get(0).getZ() > Units
					.inchesToMeters(1)) { //if another game piece is off the ground, properly update the simArray
				SimGamePiece.intake(SimGamePiece.closestPieceIndex - 1);
			} else {
				SimGamePiece.intake(SimGamePiece.closestPieceIndex);
			}
		}*/
		close = false;
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
