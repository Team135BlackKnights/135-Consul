package frc.robot.utils.drive;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoPilotAlign;
import frc.robot.commands.drive.DriveAndAimToRotation;
import frc.robot.commands.drive.DriveToLine;
import frc.robot.subsystems.drive.DrivetrainS;
import java.lang.Double;
public class PathFinder {
	public static Command goToAutoPilotPose(LocalADStarAK adStar, APTarget target, DrivetrainS drive,
			Supplier<PathConstraints> constraints, double beelineMeters, double tolerance) {
		return new AutoPilotAlign(adStar, target, drive, 2).until(() -> {
			return drive.getPose().getTranslation().getDistance(target.getReference().getTranslation()) < beelineMeters;
		}).andThen(
				new DriveAndAimToRotation(drive, target.getReference(), constraints, (Supplier<Double>)() -> tolerance)).until(
						() -> drive.getPose().getTranslation()
								.getDistance(target.getReference().getTranslation()) < tolerance);
	}

	/**
	 * Goes to a given pose with the speed constraints, and will ALWAYS end
	 * facing the given degree.
	 * 
	 * @param pose        desired end position, already on the correct side.
	 * @param constraints PathConstraints containing max speeds and velocities
	 * @param drive       drivetrain type.
	 * @return pre-made command
	 */
	public static Command goToPose(Pose2d pose,
			Supplier<PathConstraints> constraints, DrivetrainS drive,
			boolean isAuto, double endVelocity, double tolerance, double innerTolerance) {
		if (isAuto) { // skip accuracy for speed

			return AutoBuilder.pathfindToPose(pose, constraints.get(), endVelocity)
					/*
					 * Commands.defer(() -> {
					 * return AutoBuilder.pathfindToPose(pose, constraints.get(), endVelocity);
					 * }, Set.of(drive))
					 */
					.finallyDo(() -> {
						RobotContainer.field.getObject("target pose")
								.setPose(new Pose2d(-50, -50, new Rotation2d()));
						RobotContainer.closestChoreoPath = "";
						// CommandScheduler.getInstance().cancel(currentlyRunningCommand);
						// CommandScheduler.getInstance().removeComposedCommand(currentlyRunningCommand);
					}); // the void

		}
		return Commands.defer(() -> {
			return AutoBuilder.pathfindToPose(pose, constraints.get(), endVelocity);
		}, Set.of(drive))
				.until(
						() -> drive.getPose().getTranslation()
								.getDistance(pose.getTranslation()) < tolerance)
				.andThen(
						new DriveAndAimToRotation(drive, pose, constraints))
				.until(
						() -> drive.getPose().getTranslation()
								.getDistance(pose.getTranslation()) < innerTolerance)
				.finallyDo(() -> {
					RobotContainer.field.getObject("target pose")
							.setPose(new Pose2d(-50, -50, new Rotation2d()));
					RobotContainer.closestChoreoPath = "";
					// CommandScheduler.getInstance().cancel(currentlyRunningCommand);
					// CommandScheduler.getInstance().removeComposedCommand(currentlyRunningCommand);
				}); // the void
	}

	/**
	 * Goes to a given line with the speed constraints, and will ALWAYS end
	 * facing the given degree.
	 * 
	 * @param pose        desired end line
	 * @param constraints PathConstraints containing max speeds and velocities
	 * @param drive       drivetrain type.
	 * @return pre-made command
	 */
	public static Command goToLine(DrivetrainS drive, Supplier<Translation2d> pointA, Supplier<Translation2d> pointB,
			double outerTolerance,
			double innerTolerance, double humanPlayerWaitTime, Supplier<Rotation2d> rotationGoal,
			Supplier<PathConstraints> constraints, Supplier<String> corner) {
		Supplier<Translation2d> goalPoint = () -> DriveToLine.getClosestPoint(drive.getPose().getTranslation(),
				pointA.get(), pointB.get());
		return Commands.defer(() -> {
			return AutoBuilder
					.pathfindToPose(new Pose2d(goalPoint.get(), rotationGoal.get()), constraints.get(), 0);
		}, Set.of(drive))
				.until(() -> RobotContainer.drivetrainS.getLookAheadPose().getTranslation()
						.getDistance(goalPoint.get()) < outerTolerance)
				.andThen(new DriveToLine(drive, pointA, pointB, innerTolerance, humanPlayerWaitTime, rotationGoal,
						() -> ""))
				.finallyDo(() -> {
					RobotContainer.field.getObject("target pose")
							.setPose(new Pose2d(-50, -50, new Rotation2d()));
					RobotContainer.closestChoreoPath = "";
					// force drive to be un required
					//// CommandScheduler.getInstance().cancel(currentlyRunningCommand);
					// CommandScheduler.getInstance().removeComposedCommand(currentlyLineFollowingCommand);
				}); // the void

	}

	/**
	 * Goes to a given line with the speed constraints, and will ALWAYS end
	 * facing the given degree.
	 * 
	 * @param pose        desired end line
	 * @param constraints PathConstraints containing max speeds and velocities
	 * @param drive       drivetrain type.
	 * @return pre-made command
	 */
	public static Command goToLine(DrivetrainS drive, Supplier<Translation2d> pointA, Supplier<Translation2d> pointB,
			double outerTolerance, Supplier<Rotation2d> rotationGoal, Supplier<PathConstraints> constraints) {
		Supplier<Translation2d> goalPoint = () -> DriveToLine.getClosestPoint(drive.getPose().getTranslation(),
				pointA.get(), pointB.get());
		return Commands.defer(() -> {
			return AutoBuilder
					.pathfindToPose(new Pose2d(goalPoint.get(), rotationGoal.get()), constraints.get(), 0);
		}, Set.of(drive))
				.finallyDo(() -> {
					RobotContainer.field.getObject("target pose")
							.setPose(new Pose2d(-50, -50, new Rotation2d()));
					RobotContainer.closestChoreoPath = "";
					// force drive to be un required
					// CommandScheduler.getInstance().cancel(currentlyRunningCommand);
					// CommandScheduler.getInstance().removeComposedCommand(currentlyLineFollowingCommand);
				}); // the void

	}

	public static List<Pose2d> parseAutoToPose2dList(String autoFileName) {
		List<Pose2d> poseList = new ArrayList<>();
		try {
			// Read the AUTO file
			JSONObject autoJson = readJsonFromFile(autoFileName);
			// Parse the commands in the "commands" section
			JSONObject command = (JSONObject) autoJson.get("command");
			JSONObject commands = (JSONObject) command.get("data");
			JSONArray commandsList = (JSONArray) commands.get("commands");
			parseCommands(commandsList, poseList);
		} catch (Exception e) {
			System.err.println("NULL/BAD AUTO DETECTED FOR " + autoFileName);
		}
		return poseList;
	}

	private static JSONObject readJsonFromFile(String fileName)
			throws Exception {
		try (BufferedReader br = new BufferedReader(
				new FileReader(new File(Filesystem.getDeployDirectory(),
						"pathplanner/autos/" + fileName + ".auto")))) {
			StringBuilder fileContentBuilder = new StringBuilder();
			String line;
			while ((line = br.readLine()) != null) {
				fileContentBuilder.append(line);
			}
			String fileContent = fileContentBuilder.toString();
			return (JSONObject) new JSONParser().parse(fileContent);
		}
	}

	private static void parseCommands(JSONArray commands,
			List<Pose2d> poseList) {
		for (int i = 0; i < commands.size(); i++) {
			JSONObject command = (JSONObject) commands.get(i);
			String commandType = (String) command.get("type");
			JSONObject commandData = (JSONObject) command.get("data");
			switch (commandType) {
				case "sequential":
				case "deadline":
				case "parallel":
				case "race":
					JSONArray nestedCommands = (JSONArray) commandData.get("commands");
					parseCommands(nestedCommands, poseList);
					break;
				case "path":
					String pathName = (String) commandData.get("pathName");
					PathPlannerPath path;
					try {
						path = PathPlannerPath
								.fromChoreoTrajectory(pathName);
						poseList.addAll(path.getPathPoses());
					} catch (FileVersionException | IOException | ParseException e) {
						e.printStackTrace();
					}
					break;
				case "named":
				case "wait":
					// Handle named or wait commands if necessary
					break;
				default:
					throw new IllegalArgumentException(
							"Unknown command type: " + commandType);
			}
		}
	}
}
