package frc.robot.utils.Touchboard;

import org.wpilib.math.geometry.*;
import org.wpilib.math.util.Units;
import org.wpilib.command2.*;

import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import frc.robot.Robot;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.AutoTime;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class TouchboardAutoFactory {
    private final LocalADStarAK pathFinder;
    private final DrivetrainS drivetrainS;

    private final Supplier<Command> intakeCommandSupplier;
    private final Set<Subsystem> intakeRequirements;

    private final Supplier<Command> shootCommandSupplier;
    private final Set<Subsystem> shootRequirements;

    private final LoggableTunedNumber shootRadiusMeters = new LoggableTunedNumber("Touchboard/shootRadiusMeters", 2.5,
            TuningConstants.isTuningMacros);
    private final LoggableTunedNumber fieldEdgeMarginMeters = new LoggableTunedNumber(
            "Touchboard/fieldEdgeMarginMeters", 0.5, TuningConstants.isTuningMacros);

    // 135° window centered "down" from hub (toward -Y on blue)
    private double windowCenterRad = Math.PI;
    private final double windowHalfSpanRad = Math.toRadians(120.0 / 2.0);

    public TouchboardAutoFactory(
            LocalADStarAK pathFinder,
            DrivetrainS drivetrainS,
            Supplier<Command> intakeCommandSupplier,
            Set<Subsystem> intakeRequirements,
            Supplier<Command> shootCommandSupplier,
            Set<Subsystem> shootRequirements

    ) {
        this.pathFinder = pathFinder;
        this.drivetrainS = drivetrainS;
        this.intakeCommandSupplier = intakeCommandSupplier;
        this.intakeRequirements = intakeRequirements;
        this.shootCommandSupplier = shootCommandSupplier;
        this.shootRequirements = shootRequirements;
    }

    public Command build(TouchboardAutoPlan plan) {
        List<TouchboardAutoPlan.Step> steps = (plan.steps == null) ? List.of() : plan.steps;

        Command seq = Commands.print("Starting Touchboard Plan v" + plan.version);

        for (int i = 0; i < steps.size(); i++) {
            TouchboardAutoPlan.Step step = steps.get(i);
            TouchboardAutoPlan.Step next = (i + 1 < steps.size()) ? steps.get(i + 1) : null;

            if (step.atSecondsRemaining != null) {
                seq = seq.andThen(holdUntilTimeRemaining(step.atSecondsRemaining));
            }

            Set<Subsystem> reqs = new java.util.HashSet<>();
            reqs.add(drivetrainS);
            if (intakeRequirements != null) reqs.addAll(intakeRequirements);
            if (shootRequirements != null) reqs.addAll(shootRequirements);
            seq = seq.andThen(Commands.print("Running " + step.type)
                .andThen(Commands.defer(() ->commandForStep(step, next), reqs)));

            // If this step was a "return trench" (NOT followed by INTAKE) and next step is
            // timed,
            // we want to sit + shoot until the next scheduled step.
            if (step.type == TouchboardAutoPlan.StepType.TRENCH
                    && (next == null || next.type != TouchboardAutoPlan.StepType.INTAKE)
                    && next != null && next.atSecondsRemaining != null) {
                seq = seq.andThen(holdUntilTimeRemaining(next.atSecondsRemaining));
            }
        }

        return seq;
    }

    private Command commandForStep(TouchboardAutoPlan.Step step, TouchboardAutoPlan.Step next) {
        if (step == null || step.type == null)
            return Commands.none();

        switch (step.type) {
            case INTAKE:
                // run intake until timeRemaining < untilSecondsRemaining
                if (step.untilSecondsRemaining == null) {
                    return deferIntake();
                }
                return Commands.deadline(
                        Commands.waitUntil(
                                () -> 
                                     AutoTime.remaining() <= step.untilSecondsRemaining),
                        deferIntake());
            case TRENCH: {
                TouchboardAutoPlan.TrenchChoice choice = (step.choice == null) ? TouchboardAutoPlan.TrenchChoice.BEST
                        : step.choice;

                // If next step is INTAKE, we aint gotta worry
                boolean goingToIntake = (next != null && next.type == TouchboardAutoPlan.StepType.INTAKE);

                if (goingToIntake) {
                    Pose2d trenchEntrance = trenchEntrancePose(resolveTrenchChoice(choice)).plus(new Transform2d(Robot.isRed ? -1 : 1,0,new Rotation2d()));
                    return goToNoHardLineup(trenchEntrance, 4.0, new Rotation2d(Math.PI / 2.0),
                            Units.inchesToMeters(6));
                }

                // Otherwise: return trench, then stop at a safe shoot point
                    Pose2d trenchEntrance = trenchEntrancePose(resolveTrenchChoice(choice)).plus(new Transform2d(Robot.isRed ? 1 : -1,0,new Rotation2d()));
                Pose2d stopPose = pickSafeHubShootStopPose(drivetrainS.getPose());
                Logger.recordOutput("Auto/SafeStopPose", stopPose);
                Logger.recordOutput("Auto/Trench", trenchEntrance);
                return goToNoHardLineup(trenchEntrance, 4.0, new Rotation2d(-Math.PI / 2.0), Units.inchesToMeters(6))
                        .andThen(goToNoHardLineup(stopPose, 0, new Rotation2d(),
                                Units.inchesToMeters(6)));
            }

            case CHUTE: {
                // TODO
                Pose2d p = chutePose();
                return goToHardLineup(p, 3.0, new Rotation2d(Math.PI / 2.0), Units.inchesToMeters(16));
            }

            case DEPOT: {
                // TODO
                Pose2d p = depotPose();
                return goToHardLineup(p, 3.0, new Rotation2d(Math.PI / 2.0), Units.inchesToMeters(16));
            }

            case HANG: {
                // TODO
                Pose2d p = hangPose();
                return goToHardLineup(p, 3.0, new Rotation2d(Math.PI / 2.0), Units.inchesToMeters(16));
            }

            default:
                return Commands.none();
        }
    }

    /**
     * Stop drivetrain + shoot (only really matters when you’re in alliance zone)
     * until match time <= tRem
     */
    private Command holdUntilTimeRemaining(double tRem) {
        Command wait = Commands.waitUntil(() -> AutoTime.isRunning() && AutoTime.remaining() <= tRem);

        Command stopDrive = Commands.run(drivetrainS::stopModules, drivetrainS);

        Command shootGate = Commands.either(
                deferShoot(),
                Commands.none(),
                this::inAllianceZone);

        return Commands.deadline(wait, stopDrive.alongWith(shootGate));
    }

    private Pose2d trenchEntrancePose(TouchboardAutoPlan.TrenchChoice choice) {
        Translation2d t = (choice == TouchboardAutoPlan.TrenchChoice.LEFT)
                ? GeomUtil.apply(FieldConstants.LeftTrench.openingCenter, false)
                : GeomUtil.apply(FieldConstants.RightTrench.openingCenter, false);
        return new Pose2d(t.getX(), t.getY(), new Rotation2d());
    }

    private Pose2d chutePose() {
        // Outpost is at x=0 (wall). Offset so we don't target an impossible wall point.
        Translation2d t = GeomUtil.apply(FieldConstants.Outpost.centerPoint, false);
        double x = t.getX() + Units.inchesToMeters(18);
        return new Pose2d(x, t.getY(), Rotation2d.fromDegrees(0));
    }

    private Pose2d depotPose() {
        // depotCenter is Translation3d; use its x/y and keep heading 0
        var d = GeomUtil.apply(FieldConstants.Depot.depotCenter, false);
        Logger.recordOutput("Auto/DepotPose", new Pose2d(d.getX(), d.getY(), Rotation2d.fromDegrees(0)));
        return new Pose2d(d.getX(), d.getY(), Rotation2d.fromDegrees(0));
    }

    private Pose2d hangPose() {
        Translation2d t = GeomUtil.apply(FieldConstants.Tower.centerPoint, false);
        return new Pose2d(t.getX(), t.getY(), Rotation2d.fromDegrees(0));
    }

    private Pose2d pickSafeHubShootStopPose(Pose2d robotPose) {
        Translation2d hub = GeomUtil.apply(FieldConstants.Hub.innerCenterPoint, false).toTranslation2d();

        double radius = shootRadiusMeters.get();
        double margin = fieldEdgeMarginMeters.get();

        Translation2d robot = robotPose.getTranslation();

        Translation2d best = null;
        double bestDist = Double.POSITIVE_INFINITY;

        // Sample points along a 135 (lol) arc
        if (Robot.isRed) {
            // flip window center rad
            windowCenterRad += Math.PI; // up
        }
        int samples = 41;
        for (int i = 0; i < samples; i++) {
            double frac = (samples == 1) ? 0.0 : (i / (double) (samples - 1));
            double ang = (windowCenterRad - windowHalfSpanRad) + (2.0 * windowHalfSpanRad * frac);

            Translation2d candidate = new Translation2d(
                    hub.getX() + radius * Math.cos(ang),
                    hub.getY() + radius * Math.sin(ang));
            if (!isSafeCandidate(candidate, margin))
                continue;

            double dist = candidate.getDistance(robot);
            if (dist < bestDist) {
                bestDist = dist;
                best = candidate;
            }
        }

        if (best == null) {
            best = new Translation2d(hub.getX(), hub.getY() - radius);
        }

        return new Pose2d(best.getX(), best.getY(), new Rotation2d());
    }

    private boolean isSafeCandidate(Translation2d p, double margin) {
        if (p.getX() < margin)
            return false;
        if (p.getY() < margin)
            return false;
        if (p.getX() > FieldConstants.FIELD_WIDTH - margin)
            return false;
        if (p.getY() > FieldConstants.FIELD_HEIGHT - margin)
            return false;
        //NOT inside the trench/bump
        if (GeomUtil.applyX(p.getX()) > FieldConstants.LeftTrench.openingCenter.getX() &&

                GeomUtil.applyX(p.getX()) < FieldConstants.RightTrench.openingCenter.getX() &&
                p.getY() > FieldConstants.LeftTrench.openingCenter.getY() - Units.inchesToMeters(12)) {
            return false;
        }

        return true;
    }

    private TouchboardAutoPlan.TrenchChoice resolveTrenchChoice(TouchboardAutoPlan.TrenchChoice choice) {
        if (choice != TouchboardAutoPlan.TrenchChoice.BEST)
            return choice;

        // BEST = whichever trench entrance is closer
        Pose2d robot = drivetrainS.getPose();
        Translation2d r = robot.getTranslation();

        double dR = r.getDistance(GeomUtil.apply(FieldConstants.RightTrench.openingCenter, false));
        double dL = r.getDistance(GeomUtil.apply(FieldConstants.LeftTrench.openingCenter, false));

        return (dL < dR) ? TouchboardAutoPlan.TrenchChoice.LEFT : TouchboardAutoPlan.TrenchChoice.RIGHT;
    }

    private Command deferIntake() {
        return Commands.defer(intakeCommandSupplier, intakeRequirements);
    }

    private Command deferShoot() {
        return Commands.defer(shootCommandSupplier, shootRequirements);
    }

    private Command goToNoHardLineup(Pose2d targetPoseBlue, double vel, Rotation2d entryAngle, double endDistMeters) {
        return Commands.defer(
                () -> PathFinder.goToPose(
                        GeomUtil.apply(targetPoseBlue, false),
                        () -> DriveConstants.pathConstraints,
                        drivetrainS, true, vel, endDistMeters, endDistMeters),
                Set.of(drivetrainS));
    }

    private Command goToHardLineup(Pose2d targetPoseBlue, double vel, Rotation2d entryAngle, double endDistMeters) {
        return Commands.defer(
                () -> PathFinder.goToPose(
                        GeomUtil.apply(targetPoseBlue, false),
                        () -> DriveConstants.pathConstraints,
                        drivetrainS, false, vel, Math.max(1, endDistMeters), endDistMeters),
                Set.of(drivetrainS));
    }

    private boolean inAllianceZone() {
        return GeomUtil.applyX(drivetrainS.getPose().getX()) < 4.4;
    }
}
