package frc.robot.commands.drive;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.TriConsumer;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;

/**
 * AutoPilotAlign
 *
 * - If pathSupplier returns null or <2 poses, falls back to the single APTarget
 * passed in.
 * - Tune lookaheadMeters (2.0 recommended start).
 * - If your APTarget constructor signature differs, adjust
 * makeAPTargetFromPose(...) (marked).
 */
public class AutoPilotAlign extends Command {
    private final LocalADStarAK adStar;
    private final TriConsumer<PathConstraints, GoalEndState, Pose2d> pathConsumer;
    private final APTarget m_finalTarget;
    private final GoalEndState goalEndState;
    private final DrivetrainS m_drivetrain;
    private final double lookaheadMeters;
    private boolean isFinished = false;
    private AimToRotation thetaControllerCommand;
    private Rotation2d desiredRotation;
    private boolean currentlyCloseToTrench = false;

    private final static LoggableTunedNumber trenchCircleDistance = new LoggableTunedNumber("AutoPilot/TrenchCircleDistance", 2.0, TuningConstants.isTuningMacros);
    public AutoPilotAlign(LocalADStarAK adStar, APTarget finalTarget, DrivetrainS drivetrain, double lookaheadMeters) {
        this.adStar = adStar;
        this.pathConsumer = adStar.pathConsumer();
        this.m_finalTarget = finalTarget;
        this.m_drivetrain = drivetrain;
        this.goalEndState = new GoalEndState(finalTarget.getVelocity(), finalTarget.getReference().getRotation());
        this.lookaheadMeters = lookaheadMeters;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        isFinished = false;
        RobotContainer.userDrive = false;

        desiredRotation = m_drivetrain.getRotation2d();
        thetaControllerCommand = new AimToRotation(() -> desiredRotation, m_drivetrain, DriveConstants.pathConstraints);
        thetaControllerCommand.initialize();

        // Set goal ONCE (unless the goal actually changes)
        adStar.setGoalPosition(m_finalTarget.getReference().getTranslation());
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getLookAheadPose();
        ChassisSpeeds currentRobotRelative = m_drivetrain.getChassisSpeeds();
        adStar.setStartPosition(robotPose.getTranslation());
        // get path from supplier
        pathConsumer.accept(DriveConstants.pathConstraints, goalEndState, m_finalTarget.getReference());
        List<Pose2d> path = adStar.cachedPath;

        APTarget activeTarget;
        if (path == null || path.size() < 2) {
            activeTarget = m_finalTarget;
        } else {
            // compute lookahead pose and tangent
            Pose2d lookaheadPose = computeLookaheadPose(path, robotPose, lookaheadMeters);
            Rotation2d tangent = pathTangentAt(path, lookaheadPose);
            // create an APTarget around this lookahead (adapt constructor below if needed)
            activeTarget = makeAPTargetFromPose(lookaheadPose, Optional.of(tangent), m_finalTarget);
        }

        Rotation2d maskedRot = activeTarget.getReference().getRotation();
        Translation2d fieldRelativeVel = new Translation2d(
                currentRobotRelative.vxMetersPerSecond,
                currentRobotRelative.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());
        Translation2d maskedRobotRelativeVel = fieldRelativeVel.rotateBy(maskedRot.unaryMinus());

        ChassisSpeeds maskedRobotRelative = new ChassisSpeeds(
                maskedRobotRelativeVel.getX(),
                maskedRobotRelativeVel.getY(),
                currentRobotRelative.omegaRadiansPerSecond);
        Pose2d maskedPose = new Pose2d(
                robotPose.getTranslation(),
                maskedRot);
        //if the robot pose is within a trench of 2 meters, we use the Tight profile, otherwise, we use the fast profile
        if (closeToATrench() && !currentlyCloseToTrench)  {
            ((Swerve)m_drivetrain).updateAutoProfile(DriveConstants.AutopilotConstants.kTightProfile);
            System.out.println("Using tight profile");
            currentlyCloseToTrench = true;
        } else if (!closeToATrench() && currentlyCloseToTrench) {
            ((Swerve)m_drivetrain).updateAutoProfile(DriveConstants.AutopilotConstants.kFastProfile);
            currentlyCloseToTrench = false;
        }
        APResult out = ((Swerve)m_drivetrain).autopilot.calculate(maskedPose, maskedRobotRelative, activeTarget);

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(out.vx().baseUnitMagnitude(),
                out.vy().baseUnitMagnitude(), 0.0);
        ChassisSpeeds robotRelativeFromField = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
                m_drivetrain.getRotation2d());

        desiredRotation = goalEndState.rotation();
        thetaControllerCommand.execute();

        // System.out.println("Execution time: " + (endTime - startTime) + " ms");

        m_drivetrain
                .setChassisSpeeds(robotRelativeFromField.plus(new ChassisSpeeds(0, 0, RobotContainer.angularSpeed)));
        Logger.recordOutput("RobotState/ActiveAutopilotTarget", activeTarget.getReference());
        Logger.recordOutput("RobotState/EndAutopilotTarget", m_finalTarget.getReference());
        if (((Swerve)m_drivetrain).autopilot.atTarget(m_drivetrain.getPose(), m_finalTarget) && thetaControllerCommand.atGoal()) {
            isFinished = true;
        }
    }
    private boolean closeToATrench() {
        Pose2d robotPose = m_drivetrain.getPose();
        Translation2d pos = robotPose.getTranslation();
        // check if we're within 2 meters of either trench
        Translation2d[] trenchCenters = {
            FieldConstants.LeftTrench.openingCenter,
            FieldConstants.RightTrench.openingCenter,
            // opposite side, too
            GeomUtil.apply(FieldConstants.LeftTrench.openingCenter, true),
            GeomUtil.apply(FieldConstants.RightTrench.openingCenter,true)
        }; 
        double distanceThreshold = trenchCircleDistance.get();
        return pos.getDistance(trenchCenters[0]) < distanceThreshold || pos.getDistance(trenchCenters[1]) < distanceThreshold || pos.getDistance(trenchCenters[2]) < distanceThreshold || pos.getDistance(trenchCenters[3]) < distanceThreshold;
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopModules();
        thetaControllerCommand.end(interrupted);
        RobotContainer.angleOverrider = Optional.empty();
        RobotContainer.angularSpeed = 0;
        RobotContainer.userDrive = true;
        Logger.recordOutput("RobotState/ActiveAutopilotTarget", new Pose2d(-50, -50, new Rotation2d()));
    }

    /**
     * Interpolate along path poses to find a pose approx lookaheadMeters ahead of
     * the closest point.
     */
    private Pose2d computeLookaheadPose(List<Pose2d> path, Pose2d robot, double lookahead) {
        // find index of closest pose
        int closestIdx = 0;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < path.size(); i++) {
            double d = path.get(i).getTranslation().getDistance(robot.getTranslation());
            if (d < bestDist) {
                bestDist = d;
                closestIdx = i;
            }
        }

        // accumulate forward distance until >= lookahead
        double acc = 0.0;
        for (int i = closestIdx; i < path.size() - 1; i++) {
            Translation2d a = path.get(i).getTranslation();
            Translation2d b = path.get(i + 1).getTranslation();
            double seg = a.getDistance(b);
            if (acc + seg >= lookahead) {
                double remain = lookahead - acc;
                double t = (seg == 0.0) ? 0.0 : (remain / seg);
                // linear interp for translation & use heading interpolation for rotation
                Translation2d interpTrans = a.plus(b.minus(a).times(t));
                Rotation2d r1 = path.get(i).getRotation();
                Rotation2d r2 = path.get(i + 1).getRotation();
                Rotation2d interpRot = new Rotation2d(
                        MathUtil.interpolate(r1.getCos(), r2.getCos(), t),
                        MathUtil.interpolate(r1.getSin(), r2.getSin(), t));
                return new Pose2d(interpTrans, interpRot);
            } else {
                acc += seg;
            }
        }

        // if we ran out of path, return final pose
        return path.get(path.size() - 1);
    }

    private Rotation2d pathTangentAt(List<Pose2d> path, Pose2d pose) {
        // find the segment whose midpoint is closest
        int bestIdx = 0;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d mid = path.get(i).getTranslation().plus(path.get(i + 1).getTranslation()).div(2.0);
            double d = mid.getDistance(pose.getTranslation());
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        Translation2d dir = path.get(Math.min(bestIdx + 1, path.size() - 1)).getTranslation()
                .minus(path.get(bestIdx).getTranslation());
        return dir.getAngle();
    }

    private APTarget makeAPTargetFromPose(Pose2d lookaheadPose, Optional<Rotation2d> entryAngle, APTarget fallback) {
        double desiredEndV = fallback.getVelocity();

        Rotation2d entryAng = entryAngle.orElse(new Rotation2d(0.0));
        Distance rotationRadius = fallback.getRotationRadius().orElse(Units.Centimeters.of(10));
        return new APTarget(lookaheadPose).withEntryAngle(entryAng).withVelocity(desiredEndV)
                .withRotationRadius(rotationRadius);
    }
}
