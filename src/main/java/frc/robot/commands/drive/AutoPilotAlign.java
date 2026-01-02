package frc.robot.commands.drive;
import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.utils.drive.DriveConstants;



public class AutoPilotAlign extends Command {
  private final APTarget m_target;
  private final DrivetrainS m_drivetrain;

  private boolean isFinished = false;
  private AimToRotation thetaControllerCommand;
  private final PathConstraints constraints;
  private Rotation2d desiredRotation;
public AutoPilotAlign(APTarget target, DrivetrainS drivetrain) {
    m_target = target;
    m_drivetrain = drivetrain;
    this.constraints = DriveConstants.pathConstraints;
    addRequirements(drivetrain);
  }
  public AutoPilotAlign(APTarget target, DrivetrainS drivetrain, PathConstraints constraints) {
    m_target = target;
    m_drivetrain = drivetrain;
    this.constraints = constraints;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    isFinished = false;
	RobotContainer.userDrive = false;
    desiredRotation = m_drivetrain.getRotation2d();
	thetaControllerCommand = new AimToRotation(() -> desiredRotation, m_drivetrain, constraints);
    thetaControllerCommand.initialize();
  }

  @Override
  public void execute() {
    Pose2d pose = m_drivetrain.getLookAheadPose(); //may need to be getPose()
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getChassisSpeeds(),pose.getRotation());
	RobotContainer.userDrive = false;

    APResult out = Swerve.autopilot.calculate(pose, robotRelativeSpeeds, m_target);
    //out is field relative speeds, setChassisSpeeds wants robot relative
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
        out.vx().baseUnitMagnitude(),
        out.vy().baseUnitMagnitude(),
        0 //theta handled separately
    );
    ChassisSpeeds robotRelativeSpeedsFromField = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds,
        m_drivetrain.getRotation2d()
    );
    //use theta controller for rotation
    desiredRotation = out.targetAngle();
    thetaControllerCommand.execute();
    //saves to RobotContainer.angularSpeed
    m_drivetrain.setChassisSpeeds(
        robotRelativeSpeedsFromField.plus(new ChassisSpeeds(0,0,RobotContainer.angularSpeed))
    );
    if (Swerve.autopilot.atTarget(m_drivetrain.getPose(), m_target) && 
        thetaControllerCommand.atGoal()) {
      isFinished = true;
    }
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
  }
}