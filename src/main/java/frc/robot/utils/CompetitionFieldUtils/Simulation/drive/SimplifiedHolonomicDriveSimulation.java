package frc.robot.utils.CompetitionFieldUtils.Simulation.drive;

import static frc.robot.utils.maths.CommonMath.constrainMagnitude;

import java.util.function.Consumer;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.Subsystem;
import frc.robot.Robot;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveDriveSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.maths.GeometryConvertor;

import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

/**
 *
 *
 * <h1>Simulates a Simplified Holonomic Drivetrain.</h1>
 *
 * <p>This class simulates a simplified holonomic drivetrain with collision space, allowing the
 * drivetrain to drive on the field.
 *
 * <p>It is suitable for simulating AI cycle bots or opponent defense bots.
 *
 * <p>However, this drivetrain simulation is simplified and not entirely realistic—while it
 * accelerates smoothly, the kinematics are not fully accurate.
 *
 * <p>For a more realistic simulation of your robot's drivetrain, use {@link SwerveDriveSimulation}
 * instead.
 */
public class SimplifiedHolonomicDriveSimulation extends AbstractDriveTrainSimulation
    implements Subsystem {
  /**
   *
   *
   * <h2>Creates a Simplified Holonomic Drivetrain Simulation.</h2>
   *
   * @param profile the profile containing all the configuration settings for the drivetrain, see
   *     {@link
   *     frc.robot.utils.CompetitionFieldUtils.Simulation.drive.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation.DriveTrainSimulationProfile}
   * @param initialPoseOnField the initial pose of the drivetrain in the simulation world
   */
  public SimplifiedHolonomicDriveSimulation(
      DriveTrainSimulationProfile profile, Pose2d initialPoseOnField, Consumer<Pose2d> resetOdometryCallBack) {
    super(profile, initialPoseOnField, resetOdometryCallBack);
  }
  public SimplifiedHolonomicDriveSimulation(
      DriveTrainSimulationProfile profile, Pose2d initialPoseOnField, int id) {
    super(profile, initialPoseOnField, new Consumer<Pose2d>() {
      @Override
      public void accept(Pose2d t) {
      }
    });
    this.id = id;
  }
  @SuppressWarnings("unused")
  private int id;
  private ChassisVelocities desiredFieldRelativeSpeeds = new ChassisVelocities();

  /**
   *
   *
   * <h2>Runs Chassis Speeds on This Simulated Chassis.</h2>
   *
   * <p>This method sets the <strong>DESIRED</strong> robot-relative chassis speeds on the simulated
   * drivetrain.
   *
   * <p>Propelling forces will be applied to the chassis to help it accelerate smoothly to the
   * desired speeds.
   *
   * <p>This method is different from {@link
   * AbstractDriveTrainSimulation#setRobotSpeeds(ChassisVelocities)}, which jumps to the speed
   * <strong>Instantaneously</strong>.
   *
   * @param speeds the desired robot-relative speeds, represented as {@link ChassisVelocities}
   */
  public void runChassisSpeeds(ChassisVelocities speeds, boolean fieldRelative) {
    if (fieldRelative) desiredFieldRelativeSpeeds = speeds;
    else
    desiredFieldRelativeSpeeds =
    speeds.toFieldRelative(getSimulatedDriveTrainPose().getRotation());
  }
 /**
   *
   *
   * <h2>Runs Chassis Speeds on This Simulated Chassis.</h2>
   *
   * <p>This method sets the <strong>DESIRED</strong> robot-relative chassis speeds on the simulated
   * drivetrain.
   *
   * <p>Propelling forces will be applied to the chassis to help it accelerate smoothly to the
   * desired speeds.
   *
   * <p>This method is different from {@link
   * AbstractDriveTrainSimulation#setRobotSpeeds(ChassisVelocities)}, which jumps to the speed
   * <strong>Instantaneously</strong>.
   *
   * @param speeds the desired robot-relative speeds, represented as {@link ChassisVelocities}
   */
  public void runChassisSpeeds(ChassisVelocities speeds, Rotation2d fieldRelative) {
    desiredFieldRelativeSpeeds = speeds.toFieldRelative(fieldRelative);
  }
  /**
   *
   *
   * <h2>Update this chassis simulation.</h2>
   */
  @Override
  public void simulationSubTick() {
    simulateChassisBehaviorWithFieldRelativeSpeeds(desiredFieldRelativeSpeeds);
  }

  /**
   *
   *
   * <h2>Simulates the Chassis Behavior with Field-Relative Speeds.</h2>
   *
   * <p>This method updates the simulation with the given desired field-relative chassis speeds. It
   * simulates both the linear force and rotational torque due to friction and propulsion.
   *
   * @param desiredChassisSpeedsFieldRelative the desired chassis speeds relative to the field,
   *     represented as {@link ChassisVelocities}
   */
  public void simulateChassisBehaviorWithFieldRelativeSpeeds(
      ChassisVelocities desiredChassisSpeedsFieldRelative) {
    super.setAtRest(false);

    final Vector2 desiredLinearMotionPercent =
        GeometryConvertor.toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
            .multiply(1.0 / profile.maxLinearVelocity);
    simulateChassisTranslationalBehavior(
        Vector2.create(
            constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1),
            desiredLinearMotionPercent.getDirection()));

    final double desiredRotationalMotionPercent =
        desiredChassisSpeedsFieldRelative.omega / profile.maxAngularVelocity;
    simulateChassisRotationalBehavior(constrainMagnitude(desiredRotationalMotionPercent, 1));
  }

  /**
   *
   *
   * <h1>Simulates the Translational Behavior of the Chassis.</h1>
   *
   * <p>This method simulates the translational behavior of the chassis based on the desired
   * percentage of linear motion the chassis is attempting to achieve.
   *
   * @param desiredLinearMotionPercent the percentage of the linear motion the chassis is trying to
   *     reach, represented as a {@link Vector2}
   */
  private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
    final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
    final Vector2 forceVec =
        desiredLinearMotionPercent
            .copy()
            .multiply(this.profile.robotMass * this.profile.maxLinearAcceleration);

    if (robotRequestedToMoveLinearly) super.applyForce(new Force(forceVec));
    else simulateChassisLinearFriction();
  }

  /**
   *
   *
   * <h2>Simulates the Rotational Behavior of the Chassis.</h2>
   *
   * <p>This method simulates the rotational behavior of the chassis based on the desired percentage
   * of rotational motion the chassis is attempting to achieve.
   *
   * @param desiredRotationalMotionPercent the percentage of the rotational motion the chassis is
   *     trying to reach, represented as a double
   */
  private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
    final double maximumTorque = this.profile.maxAngularAcceleration * super.getMass().getInertia();
    super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
    simulateChassisAngularFriction(desiredRotationalMotionPercent);
  }

  public Command followTrajectory(
      Trajectory trajectory,
      Rotation2d startingRotation,
      Rotation2d endingRotation,
      boolean teleportToStartingPose) {
    final Timer trajectoryTimer = new Timer();
    final PIDController xController = new PIDController(5.0, 0, 0.02);
    final PIDController yController = new PIDController(5.0, 0, 0.02);
    final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            5.0,
            0,
            0.02,
            new TrapezoidProfile.Constraints(
                profile.maxAngularVelocity, profile.maxAngularAcceleration));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    final SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(Commands.runOnce(trajectoryTimer::start));
    if (teleportToStartingPose)
      commandGroup.addCommands(
          Commands.runOnce(
              () ->
                  setSimulationWorldPose(
                      new Pose2d(trajectory.getInitialPose().getTranslation(), startingRotation))));
    commandGroup.addCommands(
        Commands.run(
            () -> {
              var currentPose = getSimulatedDriveTrainPose();
              var desiredState = trajectory.sample(trajectoryTimer.get());
              var desiredHeading =
                  startingRotation.interpolate(
                      endingRotation,
                      trajectoryTimer.get() / trajectory.getTotalTime());
              var desiredFieldVelocity =
                  new ChassisVelocities(
                      desiredState.velocity * desiredState.pose.getRotation().getCos()
                          + xController.calculate(currentPose.getX(), desiredState.pose.getX()),
                      desiredState.velocity * desiredState.pose.getRotation().getSin()
                          + yController.calculate(currentPose.getY(), desiredState.pose.getY()),
                      thetaController.calculate(
                          currentPose.getRotation().getRadians(), desiredHeading.getRadians()));
              this.runChassisSpeeds(desiredFieldVelocity, true);
            }));
    return commandGroup;
  }
  private void setPathplannerChassisSpeeds(ChassisVelocities speeds, DriveFeedforwards feedforwards) {
    runChassisSpeeds(speeds, true);}
    @SuppressWarnings("unused")
   private Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                this::getSimulatedDriveTrainPose,
                this::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                this::setPathplannerChassisSpeeds,
                DriveConstants.mainController,
                DriveConstants.mainConfig,
                () -> Robot.isRed,this
        );
    }
@Override
public Pose2d getObjectOnFieldPose2d() {
	 return getSimulatedDriveTrainPose();
}
public ChassisVelocities getMeasuredChassisSpeedsRobotRelative() {
   return getMeasuredChassisSpeedsFieldRelative()
       .toRobotRelative(getObjectOnFieldPose2d().getRotation());
}

public ChassisVelocities getMeasuredChassisSpeedsFieldRelative() {
  return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(),
          getAngularVelocity());
}
}
