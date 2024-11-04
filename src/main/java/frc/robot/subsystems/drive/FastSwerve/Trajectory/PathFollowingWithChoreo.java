package frc.robot.subsystems.drive.FastSwerve.Trajectory;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/** Base command for following a path */
public class PathFollowingWithChoreo extends Command {
  private final Timer timer = new Timer();
  private final PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;
  private final EventScheduler eventScheduler;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  /**
   * Construct a base path following command
   *
   * @param path           The path to follow
   * @param poseSupplier   Function that supplies the current field-relative pose
   *                       of the robot
   * @param speedsSupplier Function that supplies the current robot-relative
   *                       chassis speeds
   * @param output         Output function that accepts robot-relative
   *                       ChassisSpeeds and feedforwards for
   *                       each drive motor. If using swerve, these feedforwards
   *                       will be in FL, FR, BL, BR order. If
   *                       using a differential drive, they will be in L, R order.
   *                       <p>
   *                       NOTE: These feedforwards are assuming unoptimized
   *                       module states. When you optimize your
   *                       module states, you will need to reverse the
   *                       feedforwards for modules that have been flipped
   * @param controller     Path following controller that will be used to follow
   *                       the path
   * @param robotConfig    The robot configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the
   *                       field? This will
   *                       maintain a global blue alliance origin.
   * @param requirements   Subsystems required by this command, usually just the
   *                       drive subsystem
   */
  public PathFollowingWithChoreo(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this.originalPath = path;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;
    this.eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    // Add all event scheduler requirements to this command's requirements
    var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    this.path = this.originalPath;
    // Ensure the ideal trajectory is generated
    Optional<PathPlannerTrajectory> idealTrajectory = this.path.getIdealTrajectory(this.robotConfig);
    idealTrajectory.ifPresent(traj -> this.trajectory = traj);
  }

  @Override
  public void initialize() {
    PathPlannerAuto.currentPathName = originalPath.name;

    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if (path.getIdealStartingState() != null) {
      // Check if we match the ideal starting state
      boolean idealVelocity = Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation = !robotConfig.isHolonomic
          || Math.abs(
              currentPose
                  .getRotation()
                  .minus(path.getIdealStartingState().rotation())
                  .getDegrees()) <= 30.0;
      if (idealVelocity && idealRotation) {
        // We can use the ideal trajectory
        trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
      } else {
        // Generate the trajectory
        trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
      }
    } else {
      // No ideal starting state, generate the trajectory
      trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
    }
    if (path.isChoreoPath()) {
      // Go back through the trajectory and add the feedforwards
      List<DriveFeedforwards> feedforwards;
      try {
        feedforwards = loadChoreoFeedforwards(path.name);
      } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();

        return;
      }
      for (int i = 0; i < trajectory.getStates().size(); i++) {
        if (i >= feedforwards.size()) {
          System.err.println("Not enough feedforwards for trajectory");
          break;
        }
        System.out.println("Adding feedforwards to trajectory" + feedforwards.get(i).robotRelativeForcesXNewtons()[2]);

        trajectory.getStates().get(i).feedforwards = feedforwards.get(i);
      }
    }

    PathPlannerLogging.logActivePath(path);
    PPLibTelemetry.setCurrentPath(path);

    eventScheduler.initialize(trajectory);

    timer.reset();
    timer.start();
  }

  private boolean movingRight = false;

  public List<DriveFeedforwards> loadChoreoFeedforwards(String trajectoryName)
      throws IOException, ParseException, FileVersionException {
    List<DriveFeedforwards> feedforwards = new ArrayList<>();
    // if it ends in something like mainAuto.0.traj, remove the .0
    String pathName = trajectoryName;
    // store the value of the index after the dot (using example above, this would
    // be 0)
    int dotIndex = -1;
    if (trajectoryName.contains(".")) {
      pathName = trajectoryName.substring(0, trajectoryName.indexOf("."));
      dotIndex = Character.getNumericValue(
          trajectoryName.substring(trajectoryName.indexOf(".") + 1, trajectoryName.indexOf(".") + 2).charAt(0));
      System.out.println("Dot index: " + dotIndex);
    }

    try (BufferedReader br = new BufferedReader(
        new FileReader(new File(Filesystem.getDeployDirectory(), "choreo/" + pathName + ".traj")))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }

      String fileContent = fileContentBuilder.toString();
      JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

      // Check file version compatibility
      String version = json.get("version").toString();
      String[] versions = version.split("\\.");
      if (versions.length < 2 || !versions[0].equals("v2025") || !versions[1].equals("0")) {
        throw new FileVersionException(version, "v2025.0.X", pathName + ".traj");
      }

      JSONObject trajJson = (JSONObject) json.get("trajectory");
      JSONArray splitsJson = (JSONArray) trajJson.get("splits");
      List<Integer> splits = new ArrayList<>();
      for (Object o : splitsJson) {
        splits.add(((Number) o).intValue());
      }

      if (splits.isEmpty() || splits.get(0) != 0) {
        splits.add(0, 0);
      }
      JSONArray samples = (JSONArray) trajJson.get("samples");
      if (dotIndex == -1) {
        int sampleCount = 0;
        for (Object s : samples) {
          sampleCount++;
          feedforwards.add(processSample((JSONObject) s, sampleCount));
        }
      } else {
        // Handle split case based on dotIndex
        int splitStartIdx = splits.get(dotIndex);
        int splitEndIdx = (dotIndex < splits.size() - 1) ? splits.get(dotIndex + 1) : samples.size();
        int sampleCount = 0;
        for (int i = splitStartIdx; i < splitEndIdx; i++) {
          sampleCount++;
          JSONObject sample = (JSONObject) samples.get(i);
          feedforwards.add(processSample(sample, sampleCount));
        }
      }
    }

    return feedforwards;
  }

  private DriveFeedforwards processSample(JSONObject sample, int sampleCount) {
    JSONArray moduleForcesXArray = (JSONArray) sample.get("fx");
    double[] moduleForcesX = new double[4];
    int i = 0;
    for (Object force : moduleForcesXArray) {
      moduleForcesX[i] = (((Number) force).doubleValue());
      i++;
    }
    JSONArray moduleForcesYArray = (JSONArray) sample.get("fy");
    double[] moduleForcesY = new double[4];
    i = 0;
    for (Object force : moduleForcesYArray) {
      moduleForcesY[i] = (((Number) force).doubleValue());
      i++;
    }
    double[] linearForces = new double[4];
    double xVel = ((Number) sample.get("vx")).doubleValue();
    double yVel = ((Number) sample.get("vy")).doubleValue();
    if (sampleCount == 1) {

      // Calculate the angle of the vector
      double angle = Math.atan2(moduleForcesY[0], moduleForcesX[0]);
      // if between -90 and 90, then the force is in the direction of the velocity
      if (angle > -Math.PI / 2 && angle < Math.PI / 2) {
        movingRight = true;
      } else {
        movingRight = false;
      }
    }
    for (int j = 0; j < moduleForcesX.length; j++) {
      double forceMagnitude = Math.hypot(moduleForcesX[j], moduleForcesY[j]);
      double velocityMagnitude = Math.hypot(xVel, yVel);

      // Calculate the dot product to determine if force aligns with velocity
      double dotProduct = (moduleForcesX[j] * xVel + moduleForcesY[j] * yVel);

      // Sign adjustment based on alignment with velocity direction
      double signAdjustment = Math.signum(dotProduct / (forceMagnitude * velocityMagnitude));
      if (Double.isNaN(signAdjustment)) {
        signAdjustment = 1;
      }
      // Assign the adjusted force magnitude
      linearForces[j] = forceMagnitude * signAdjustment * (movingRight ? 1 : -1);
    }
    // Assuming DriveFeedforwards constructor takes fx and fy as parameters
    return new DriveFeedforwards(
        new double[4],
        linearForces,
        new double[4],
        moduleForcesX,
        moduleForcesY);
  }

  @Override
  public void execute() {
    double currentTime = timer.get();
    var targetState = trajectory.sample(currentTime);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();
    
    ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    PPLibTelemetry.setTargetPose(targetState.pose);
    PathPlannerLogging.logTargetPose(targetState.pose);

    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
    output.accept(targetSpeeds, targetState.feedforwards);

    eventScheduler.execute(currentTime);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    PathPlannerAuto.currentPathName = "";

    // Only output 0 speeds when ending a path that is supposed to stop, this allows
    // interrupting
    // the command to smoothly transition into some auto-alignment routine
    if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
      output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    PathPlannerLogging.logActivePath(null);

    eventScheduler.end();
  }

  /**
   * Create a command to warmup on-the-fly generation, replanning, and the path
   * following command
   *
   * @return Path following warmup command
   */
  public static Command warmupCommand() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.kZero), new Pose2d(6.0, 6.0, Rotation2d.kZero));
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        new PathConstraints(4.0, 4.0, 4.0, 4.0),
        new IdealStartingState(0.0, Rotation2d.kZero),
        new GoalEndState(0.0, Rotation2d.kCW_90deg));

    return new PathFollowingWithChoreo(
        path,
        () -> Pose2d.kZero,
        ChassisSpeeds::new,
        (speeds, feedforwards) -> {
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        new RobotConfig(
            75,
            6.8,
            new ModuleConfig(
                0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
            0.55),
        () -> true)
        .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"))
        .ignoringDisable(true);
  }
}
