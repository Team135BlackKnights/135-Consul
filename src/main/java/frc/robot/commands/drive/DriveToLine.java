package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldObjects.Reefscape2025FieldObjects;
import frc.robot.utils.drive.DriveConstants;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

public class DriveToLine extends Command {
    private final DrivetrainS drive;
    private final Supplier<Translation2d> pointA;
    private final Supplier<Translation2d> pointB;
    private final double tolerance;
    private double timeout = -1;
    private final ProfiledPIDController driveController;
    private final Supplier<Rotation2d> rotationGoal;
    private final Supplier<String> corner;
    private AimToRotation thetaControllerCommand;
    private DrivetrainC userControlCommand;
    private boolean hasGottenCoral;
    private Translation2d[] possiblePoints;
    private Timer timeoutTimer = new Timer();
    private boolean isFinished= false;
    // allow live updating via LoggableTunedNumbers
    private static final LoggableTunedNumber driveKp = new LoggableTunedNumber(
            "DriveToLine/DriveKp");
    private static final LoggableTunedNumber driveKd = new LoggableTunedNumber(
            "DriveToLine/DriveKd");
    private static final LoggableTunedNumber driveTolerance = new LoggableTunedNumber(
            "DriveToLine/DriveTolerance");
    // Default the TunedNumbers on boot
    static {
        driveKp.initDefault(2, TuningConstants.isTuningMacros);
        driveKd.initDefault(0.0, TuningConstants.isTuningMacros);

        driveTolerance.initDefault(Units.inchesToMeters(1.5), TuningConstants.isTuningMacros);
    }

    public DriveToLine(DrivetrainS drive, Supplier<Translation2d> pointA, Supplier<Translation2d> pointB,
            double tolerance, Supplier<Rotation2d> rotationGoal, Supplier<String> corner) {
        this.drive = drive;
        this.pointA = pointA;
        this.pointB = pointB;
        this.corner = corner;
        this.tolerance = tolerance;
        this.rotationGoal = rotationGoal;
        this.driveController = new ProfiledPIDController(driveKp.get(), 0.0, driveKd.get(),
                new TrapezoidProfile.Constraints(DriveConstants.pathConstraints.maxVelocityMPS(), 3.0));
    }

    public DriveToLine(DrivetrainS drive, Supplier<Translation2d> pointA, Supplier<Translation2d> pointB,
            double tolerance, double timeout, Supplier<Rotation2d> rotationGoal, Supplier<String> corner) {
        this.drive = drive;
        this.pointA = pointA;
        this.pointB = pointB;
        this.corner = corner;
        this.tolerance = tolerance;
        this.timeout = timeout;
        this.rotationGoal = rotationGoal;
        this.driveController = new ProfiledPIDController(driveKp.get(), 0.0, driveKd.get(),
                new TrapezoidProfile.Constraints(DriveConstants.pathConstraints.maxVelocityMPS(), 3.0));
    }
    @Override
    public void initialize() {
        isFinished = false;
        thetaControllerCommand = new AimToRotation(rotationGoal, drive, DriveConstants.pathConstraints);
        thetaControllerCommand.initialize();
        userControlCommand = new DrivetrainC(drive);
        userControlCommand.initialize();
        hasGottenCoral = false;
        if ("redLeft".equals(corner.get())) {
            possiblePoints = FieldConstants.CoralStation.validRedLeft;
        } else if ("redRight".equals(corner.get())) {
            possiblePoints = FieldConstants.CoralStation.validRedRight;
        } else if ("blueLeft".equals(corner.get())) {
            possiblePoints = FieldConstants.CoralStation.validBlueLeft;
        } else if ("blueRight".equals(corner.get()))
            possiblePoints = FieldConstants.CoralStation.validBlueRight;
        else {
            possiblePoints = null;
        }
        if (timeout != -1 && timeout != 999){
            timeoutTimer.reset();
            timeoutTimer.start();
        }
    }

    public static Translation2d getClosestPoint(Translation2d currentPose, Translation2d start, Translation2d end) {

        Translation2d lineDirection = end.minus(start).div(end.getDistance(start));
        double dotProduct = (currentPose.getX() - start.getX()) * lineDirection.getX() +
                (currentPose.getY() - start.getY()) * lineDirection.getY();
        Translation2d projectedPoint = start.plus(lineDirection.times(dotProduct));

        // Ensure the projected point stays within the segment
        double startToProjected = start.getDistance(projectedPoint);
        double endToProjected = end.getDistance(projectedPoint);
        double startToEnd = start.getDistance(end);

        Translation2d closestPoint;
        if (startToProjected > startToEnd) {
            closestPoint = end;
        } else if (endToProjected > startToEnd) {
            closestPoint = start;
        } else {
            closestPoint = projectedPoint;
        }
        return closestPoint;
    }

    @Override
    public void execute() {
        RobotContainer.currentPath = "DRIVE_TO_LINE";
        var currentPose = drive.getLookAheadPose().getTranslation();
        var start = pointA.get();
        var end = pointB.get();
        thetaControllerCommand.execute();
        Translation2d closestPoint;
        if (possiblePoints != null) {
            double minDistance = Double.MAX_VALUE;
            closestPoint = possiblePoints[0];
            for (Translation2d point : possiblePoints) {
                double distance = currentPose.getDistance(point);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPoint = point;
                }
            }
        } else {
            closestPoint = getClosestPoint(currentPose, start, end);
        }

        double distanceToLine = currentPose.getDistance(closestPoint);

        boolean withinTolerance = distanceToLine < tolerance;
        Rotation2d angleToTarget = currentPose.minus(closestPoint).getAngle();
        double driveVelocityScalar = driveController.calculate(distanceToLine, 0.0);

        Translation2d driveVelocity = new Pose2d(new Translation2d(), angleToTarget)
                .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
                .getTranslation();
        if (!withinTolerance) {
            RobotContainer.userDrive = false;
            RobotContainer.xSpeed = 0;
            RobotContainer.ySpeed = 0;
            RobotContainer.withinLineTolerance = false;
            drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveVelocity.getX(), driveVelocity.getY(), RobotContainer.angularSpeed,
                    drive.getLookAheadPose().getRotation()));

        } else {
            if (timeoutTimer.isRunning()){
                if (timeoutTimer.hasElapsed(timeout)){
                    isFinished = true;
                }
            }
            RobotContainer.userDrive = true;
            RobotContainer.withinLineTolerance = true;
            RobotContainer.xSpeed = driveVelocity.getX();
            RobotContainer.ySpeed = driveVelocity.getY();
            userControlCommand.execute();
            // if in sim, gimme game piece
            if (Constants.currentMode == Mode.SIM && !hasGottenCoral) {
                hasGottenCoral = true;
                RobotContainer.fieldSimulation
                        .addGamePiece(new Reefscape2025FieldObjects.ReefscapeCoralOnManipulator());

            }
        }
        Logger.recordOutput("DriveToLine/DistanceError", distanceToLine);
        Logger.recordOutput("DriveToLine/DistanceSetpoint", driveController.getSetpoint().position);
        Logger.recordOutput("DriveToLine/ClosestPoint", new Pose2d(closestPoint, drive.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.currentPath = "";
        RobotContainer.withinLineTolerance = false;
        RobotContainer.userDrive = true;
        RobotContainer.xSpeed = 0;
        RobotContainer.ySpeed = 0;
        if (thetaControllerCommand != null) {
            thetaControllerCommand.end(interrupted);
        }
        RobotContainer.angleOverrider = Optional.empty();
        RobotContainer.angularSpeed = 0;

        drive.stopModules();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
