package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.GeometryConstants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.vision.VisionConstants;

public class DriveToTargetUsingDriveAndAimAtPose extends Command {
	private final DrivetrainS swerveS;
	private boolean isFinished = false;
	Timer timer = new Timer();
	boolean loaded = false;
	private DriveAndAimAtPose drivingCommand = null;
	private LoggableTunedNumber maxNoteSpeed = new LoggableTunedNumber(
			"DriveToTargetUsingDriveAndAimAtPose/maxNoteSpeed", 2.5);
	private LoggableTunedNumber intakeOffsetPoseDistance = new LoggableTunedNumber(
			"DriveToTargetUsingDriveAndAimAtPose/intakeOffsetPoseDistance",
			Units.inchesToMeters(2));
	private LoggableTunedNumber maxNoteSpeedClose = new LoggableTunedNumber(
			"DriveToTargetUsingDriveAndAimAtPose/maxNoteSpeedClose", 1.75);
	private LoggableTunedNumber closeCutoff = new LoggableTunedNumber(
			"DriveToTargetUsingDriveAndAimAtPose/closeCutoff",
			Units.inchesToMeters(15));
	private LoggableTunedNumber maxAutoLookTime = new LoggableTunedNumber(
			"DriveToTargetUsingDriveAndAimAtPose/maxAutoLookTime", 2);
	private boolean commandStarted = false;
	private double gamePieceDistance = 99; //in m
	private Rotation2d gamePieceAngleError = new Rotation2d(99); //in rad
	private Timer lastRemovedNoteTimer = new Timer();
	private Supplier<Translation2d> goalSupplier;
	private Supplier<Boolean> gamePieceCollectedSupplier;
	private Supplier<Boolean> visionOkaySupplier;
	private Translation2d[] gamePieceLocations = FieldConstants.NOTE_INITIAL_POSITIONS;
	public DriveToTargetUsingDriveAndAimAtPose(DrivetrainS swerveS,
			Supplier<Translation2d> goalSupplier,
			Supplier<Boolean> visionOkaySupplier,
			Supplier<Boolean> noteDetectedSupplier) {
		this.swerveS = swerveS;
		addRequirements(swerveS);
		this.goalSupplier = goalSupplier;
		this.visionOkaySupplier = visionOkaySupplier;
		this.gamePieceCollectedSupplier = noteDetectedSupplier;
	}


	//Example supplier for goalSupplier, make a seperate command to use this specific supplier if wanted.
	/*
	int counter = 0;
	double lastTx = 0, lastTy = 0;

	private Translation2d updateNotePose() {
		double gamePieceTx = 0, gamePieceTy = 0;
		Pose2d currentPose = swerveS.getPose();
		boolean noteDetected = false;
		if (Constants.currentMode == Mode.SIM) {
			// In simulation, get the current pose, and set the degree value to
			Translation2d targetPieceLocation = RobotContainer.fieldSimulation
					.getClosestGamePieceOnGround().getPose3d().toPose2d()
					.getTranslation();
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
			noteDetected = true;
		} else {
			// THESE ARE IN D E G R E E S
			LimelightHelpers.LimelightTarget_Detector[] results = LimelightHelpers
					.getLatestResults(
							VisionConstants.limelightName).targetingResults.targets_Detector;
			System.out.println(results);
			for (LimelightHelpers.LimelightTarget_Detector object : results) {
				if (object.confidence < .4) {
					continue;
				}
				if (object.classID == AITargets.kGamePiece.getValue()) {
					gamePieceTx = -object.tx;
					gamePieceTy = object.ty;
					noteDetected = true;
					Logger.recordOutput("Vision/NoteDetected", true);
				} else {
					noteDetected = false;
					Logger.recordOutput("Vision/NoteDetected", false);
				}
			}
		}
		if (gamePieceTx == lastTx && gamePieceTy == lastTy) {
			counter++;
		} else {
			counter = 0;
		}
		if (counter > 10) { //prevent flickering of note detection
			noteDetected = false;
		}
		if (noteDetected)
			return GeomUtil
					.calculateFieldRelativePose3d(currentPose, gamePieceTx, gamePieceTy,
							Units.inchesToMeters(
									VisionConstants.limelightLensHeightoffFloorInches),
							Units.inchesToMeters(2),
							VisionConstants.limeLightAngleOffsetDegrees)
					.getTranslation().toTranslation2d();
		return null;
	}
	 */
	

	@Override
	public void initialize() {
		isFinished = false;
		commandStarted = false;
		/*if (Constants.currentMode == Mode.SIM) {
			//If the robot is in sim, target the closest game piece to drive to
			this.targetPieceLocation = RobotContainer.fieldSimulation
					.getClosestGamePieceOnGround().getPose3d().toPose2d()
					.getTranslation();
		}*/
		gamePieceLocations = FieldConstants.NOTE_INITIAL_POSITIONS;
		timer.restart();
		lastRemovedNoteTimer.restart();
		RobotContainer.currentPath = "INTAKINGSTART_DRIVEPOSE";
		RobotContainer.userDrive = false; //stop user control
		drivingCommand = new DriveAndAimAtPose(swerveS, this::getGamePiecePose,
				maxNoteSpeed.get(), false);
		if (getGamePiecePose() != null) {
			drivingCommand.initialize();
			commandStarted = true;
		}
	}
	private Translation2d getGamePiecePose() {
		//if no note detected, pause drive command
		if (goalSupplier.get() != null) {
			return goalSupplier.get();
		} else {
			//if vision is down, failover to closest gamepiece
			if (Constants.currentMatchState == FRCMatchState.AUTO){
				if (!visionOkaySupplier.get()) {
					return FieldConstants.getClosestGamePieceFromListOfNotes(
							swerveS.getPose().getTranslation(),
							gamePieceLocations);
				}
				//vision is okay, but no gamepiece detected, check if we're within x degrees of the target note
				//if we are, then we're close enough to say the note doesn't exist, re-search for a new one WITHOUT that note in the list
				if (Math.abs(gamePieceDistance) <= VisionConstants.limelightCloseEnoughToConsiderMissingDistance.get() && Math.abs(gamePieceAngleError.getDegrees()) <= VisionConstants.limelightCloseEnoughToConsiderMissingAngle.get() && lastRemovedNoteTimer.hasElapsed(VisionConstants.limelightCloseEnoughToConsiderMissingTimeout.get())) {
					//remove the closest note from the list
					Translation2d closestNote = FieldConstants.getClosestGamePieceFromListOfNotes(
							swerveS.getPose().getTranslation(),
							gamePieceLocations);
					//find that note in the list, and remove it
					gamePieceLocations = java.util.Arrays.stream(gamePieceLocations)
							.filter(note -> !note.equals(closestNote)) //where NOT the closest note
							.toArray(Translation2d[]::new);
					lastRemovedNoteTimer.restart();
				}
				return FieldConstants.getClosestGamePieceFromListOfNotes(
					swerveS.getPose().getTranslation(),
					gamePieceLocations);
			}
			return null; //no clue where the note is. This will cause the robot to spin in place at 10% speed
		}
	}

	@Override
	public void execute() {
		//determine if game piece loaded
		RobotContainer.currentPath = "INTAKING_DRIVEPOSE";
		if (gamePieceCollectedSupplier.get()) {
			isFinished = true;
		}
		Pose2d currentPose = swerveS.getPose();
		Translation2d target = getGamePiecePose();
		Logger.recordOutput("RotateAndDriveToPose/Target",
				new Pose2d(target, new Rotation2d()));
		gamePieceDistance = GeomUtil.calculateDistanceFromTranslation2d(
				currentPose.getTranslation(), target)
				- intakeOffsetPoseDistance.get() - FieldConstants.NOTE_DIAMETER / 2;
		Rotation2d desiredAngle = GeomUtil.rotationFromCurrentToTarget(
				currentPose.getTranslation(), target, VisionConstants.driveToAITargetApproachDirection);

		gamePieceAngleError = desiredAngle.minus(currentPose.getRotation());
		Logger.recordOutput("RotateAndDriveToPose/gamePieceDistance",
				gamePieceDistance);
		boolean gamePieceDetected = target != null;
		Logger.recordOutput("RotateAndDriveToPose/NoteDetected",
				gamePieceDetected);
		if (gamePieceDistance <= GeometryConstants.ObjectDistanceZeroSpeed
				&& Constants.currentMode == Constants.Mode.SIM) { //less than x inches away, STOP!
			System.out.println("TOO CLOSE");
			isFinished = true;
		} else {
			if (gamePieceDetected && !commandStarted) {
				drivingCommand.initialize();
				commandStarted = true;
			}
			if (gamePieceDetected) {
				if (gamePieceDistance <= closeCutoff.get()) {
					drivingCommand.updateConstraints(maxNoteSpeedClose.get());
				} else {
					//if we're not close, use the normal speed
					drivingCommand.updateConstraints(maxNoteSpeed.get());
				}
				drivingCommand.execute();
			} else {
				//not detected, spin in place to find it OR until timer runs out (if in auto)
				if (Constants.currentMatchState == FRCMatchState.AUTO
						&& timer.get() > maxAutoLookTime.get()) {
					isFinished = true; //if in auto, and greater than max time, STOP ENTIRE COMMAND
				}
				swerveS.setChassisSpeeds(new ChassisSpeeds(0, 0,
						0.2 * DriveConstants.kMaxTurningSpeedRadPerSec));
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
		timer.reset();
		lastRemovedNoteTimer.stop();
		lastRemovedNoteTimer.reset();
		swerveS.stopModules();
		if (gamePieceCollectedSupplier.get()
				|| Constants.currentMode == Mode.SIM) {
			//call the "load" command to make sure our RPM ramp up doesn't spit out the note
			//intakeS.load(false);
		} else {
			//intakeS.stop();
		}
		RobotContainer.currentPath = "";
		RobotContainer.userDrive = true; //give user control
		RobotContainer.angleOverrider = Optional.empty();
		if (Constants.currentMode == Mode.SIM) {
			if (!interrupted) {
				RobotContainer.fieldSimulation.intakeNote();
			}
		}
	}

	@Override
	public boolean isFinished() { return isFinished; }
}
