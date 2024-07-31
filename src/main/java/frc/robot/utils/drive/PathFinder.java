package frc.robot.utils.drive;

import org.json.simple.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.function.Supplier;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import org.json.simple.JSONArray;
import org.json.simple.parser.JSONParser;

public class PathFinder {
	/**
	 * Goes to a given pose with the speed constraints, and will ALWAYS end
	 * facing the given degree.
	 * 
	 * @param pose        desired end position
	 * @param constraints PathConstraints containing max speeds and velocities
	 * @param drive       drivetrain type.
	 * @return pre-made command
	 */
	public static Command goToPose(Pose2d pose,
			Supplier<PathConstraints> constraints, DrivetrainS drive,
			boolean isAuto, double endVelocity) {
		if (isAuto) { //skip accuracy for speed
			return AutoBuilder
					.pathfindToPose((pose), constraints.get(), endVelocity)
					.finallyDo(() -> RobotContainer.field.getObject("target pose")
							.setPose(new Pose2d(-50, -50, new Rotation2d()))); //the void		));
		}
		return AutoBuilder.pathfindToPose((pose), constraints.get(), endVelocity)
				.andThen(new DriveToPose(drive, pose, constraints.get()))
				.finallyDo(() -> RobotContainer.field.getObject("target pose")
						.setPose(new Pose2d(-50, -50, new Rotation2d()))); //the void		));
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
		}
		catch (Exception e) {
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
				PathPlannerPath path = PathPlannerPath
						.fromChoreoTrajectory(pathName);
				poseList.addAll(path.getPathPoses());
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
