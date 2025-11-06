package frc.robot.commands.drive.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AimToRotation;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.maths.TimeUtil;
import frc.robot.utils.vision.VisionConstants;

public class AimToAprilTag extends Command {
    private final DrivetrainS drive;
    private final Vision vision;
    private final int tagId;
    private final Pose3d knownTagPose;
    private final boolean shouldUseDefaultPoseIfNotFound;
    private AimToRotation aimToRotationCommand;
    private Pose2d targetPose;

    /**
     * Aim the robot at a specific AprilTag. If the tag is not found, the robot will
     * aim at a known pose, or aim at the current heading.
     * 
     * @param drive                          The drive subsystem
     * @param vision                         The vision subsystem
     * @param tagId                          The fiducial ID of the tag to aim at
     * @param shouldUseDefaultPoseIfNotFound If true, the robot will aim at the
     *                                       known pose of the tag when it isnot
     *                                       found. If false, the robot will aim at
     *                                       the current heading (do nothing) if the
     *                                       tag is not found.
     */
    public AimToAprilTag(Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier,DrivetrainS drive, Vision vision, int tagId, boolean shouldUseDefaultPoseIfNotFound) {
        this.drive = drive;
        this.vision = vision;
        this.tagId = tagId;
        this.shouldUseDefaultPoseIfNotFound = shouldUseDefaultPoseIfNotFound;
        // Fetch the known tag pose from the layout
        this.knownTagPose = aprilTagLayoutSupplier.get().getLayout().getTagPose(tagId).orElseThrow(
                () -> new IllegalArgumentException("Invalid tag ID: " + tagId));
    }

    @Override
    public void initialize() {
        RobotContainer.currentPath = "AIMTOAPRILTAG";
        aimToRotationCommand = new AimToRotation(getAimGoal(), drive, DriveConstants.pathConstraints);
        aimToRotationCommand.initialize(); // Manually initialize the command
    }

    private Supplier<Rotation2d> getAimGoal() {
        return () -> {
            if (targetPose == null) {
                return drive.getPose().getRotation();
            }
            return GeomUtil.rotationFromCurrentToTarget(
                    drive.getPose().getTranslation(),
                    targetPose.getTranslation(),
                    GeomUtil.ApproachDirection.BACK_RIGHT);
        };
    }

    private boolean hasTarget = false;

    @Override
    public void execute() {
        // Try to get the pose from vision
        TargetObservation[] observations = vision.getLatestTargetObservations();
        hasTarget = false;
        for (int i = 0; i < observations.length; i++) {
            if (observations[i].id() == tagId
                    && Math.abs(TimeUtil.getLogTimeSeconds() - observations[i].timestamp()) < 1) {
                Pose3d cameraPose = VisionConstants.cameras[i].getPose().get();
                Pose3d fieldToCameraPose = new Pose3d(RobotContainer.drivetrainS.getPose())
                        .transformBy(
                                new Transform3d(cameraPose.getTranslation().getX(), cameraPose.getTranslation().getY(),
                                        cameraPose.getTranslation().getZ(), cameraPose.getRotation()));
                Pose3d fieldToTagPose = fieldToCameraPose.transformBy(observations[i].cameraToTarget());
                targetPose = fieldToTagPose.toPose2d();
                hasTarget = true;
                Logger.recordOutput("TagPose (Vision)", fieldToTagPose);
                break;
            }
        }
        if (!hasTarget) {
            if (shouldUseDefaultPoseIfNotFound) {
                // Fall back to the known tag pose
                targetPose = knownTagPose.toPose2d();
                Logger.recordOutput("TagPose (Fallback)", knownTagPose);
            } else {
                targetPose = null;
            }
        }
        aimToRotationCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.currentPath = "";
        aimToRotationCommand.end(interrupted); // Manually end the command
        RobotContainer.angleOverrider = Optional.empty();
        RobotContainer.angularSpeed = 0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
