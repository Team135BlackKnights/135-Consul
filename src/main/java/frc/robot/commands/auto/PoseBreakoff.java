package frc.robot.commands.auto;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;

public class PoseBreakoff extends Command{
    private final DrivetrainS drive;
    public enum BreakoffType{
        shoot,
        //TOP_SCORE, (like a pick and place game)
        //BOTTOM_SCORE
    }
    private final BreakoffType type;
    private final PathConstraints constraints;
    private final double endVelocity;
    private final boolean usePathfinder;
    private final double tolerance;
    /**
     * Go to a desisred Breakoff.
     * @param drive subsystem
     * @param type BreakoffType
     * @param constraints PathConstraints to use for path.
     * @param endVelocity end velocity of the path
     * @param usePathfinder use Pathfinder as default
     * @param tolerance if using pathfinder
     */
    public PoseBreakoff(DrivetrainS drive, BreakoffType type, PathConstraints constraints, double endVelocity, boolean usePathfinder, double tolerance){
        this.drive = drive;
        this.type = type;
        this.constraints = constraints;
        this.endVelocity = endVelocity;
        this.usePathfinder = usePathfinder;
        this.tolerance = tolerance;

    }
    private Command pathCommand;
    private Collection<Pair<String, PathPlannerPath>> readAllStartingPoses(String breakoffKeyword) {
		Collection<Pair<String, PathPlannerPath>> poses = new ArrayList<>();
	    File choreoDirectory = new File(Filesystem.getDeployDirectory(),
				"choreo/");
		for (String choreo : choreoDirectory.list()) {
			// count number of . in the name using regex
			int dotCount = choreo.split("\\.", -1).length - 1;
			if (choreo.contains(".traj") && dotCount == 1) {
				// remove the .traj from the name
				choreo = choreo.replace(".traj", "");
				try {
                    //check if the choreo file contains the keyword
                    if (choreo.contains(breakoffKeyword)){
                        PathPlannerPath startingPose = PathPlannerPath.fromChoreoTrajectory(choreo);
                        poses.add(new Pair<String, PathPlannerPath>(choreo, startingPose));
                    }
				} catch (FileVersionException | IOException | ParseException | NullPointerException e) {
					e.printStackTrace();
				}

			}
		}
		return poses;
	}
    /**
     * @voodoo  Do NOT mess with the distance calculations, they are there for a reason. If you touch them, the entire robot will become NaN. Do NOT. Touch. Them.
     */
    @Override
    public void initialize(){
        isFinished = false;
        if (usePathfinder){
            pathCommand = usePathFinder();
        }else{
            //Using Choreo precalc
            Collection<Pair<String, PathPlannerPath>> poses = readAllStartingPoses(type.toString());
            Pose2d currentPose = drive.getPose();
            double minDistance = Double.MAX_VALUE;
            Pair<String, PathPlannerPath> closestPath = null;
            for (Pair<String, PathPlannerPath> path : poses){
                double distance = path.getSecond().getStartingDifferentialPose().getTranslation().getDistance(currentPose.getTranslation());
                if (distance < minDistance){
                    minDistance = distance;
                    closestPath = path;
                }
            }
            if (closestPath != null){
                try {
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(closestPath.getSecond().getPathPoses());
                    PathPlannerPath path = new PathPlannerPath(waypoints, DriveConstants.pathConstraints,new IdealStartingState(Math.hypot(drive.getFieldVelocity().dx,drive.getFieldVelocity().dy), new Rotation2d()), closestPath.getSecond().getGoalEndState());
                    path.preventFlipping = true;
                    pathCommand = AutoBuilder.followPath(path);
                } catch (FileVersionException e) {
                    e.printStackTrace();
                    System.err.println("Error loading choreo path, using Pathfinder as fallback.");
                    pathCommand = usePathFinder();
                }
            }else{
                System.err.println("No valid choreo path found, using Pathfinder as fallback.");
                pathCommand = usePathFinder();
            }
        }
        pathCommand.initialize();
    }
    private boolean isFinished = false;
    public Command usePathFinder(){
        Pose2d desiredPose;
        if (type == BreakoffType.shoot){
            Pose2d currentPose = drive.getPose();
            Translation2d currentTranslation = currentPose.getTranslation();
            if (currentTranslation.getX() < 3){ 
                if (currentTranslation.getY() > 5)
                    desiredPose = new Pose2d(2, 6, Rotation2d.fromDegrees(135));
                else
                    desiredPose = new Pose2d(2, 3, Rotation2d.fromDegrees(45));
            }else{
                if (currentTranslation.getY() > 3)
                    desiredPose = new Pose2d(6, 3, Rotation2d.fromDegrees(180));
                else
                    desiredPose = new Pose2d(5, 2, Rotation2d.fromDegrees(160));
            }
        }else{
            //Add other breakoff types!
            System.out.println("No valid breakoff type provided, defaulting to 0,0,0");
            desiredPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }
        return PathFinder.goToPose(desiredPose, () -> constraints, drive, true, endVelocity,tolerance,tolerance);
    }
    @Override
    public void execute(){
        if (pathCommand != null) {
			if (pathCommand.isFinished()) {
				System.out.println("PoseBreakoff is finished");
				pathCommand.end(false);
				isFinished = true;
			}else{
				pathCommand.execute();
			}
		} else {
			//We've lost all sense of time. End the command.
			System.out.println("BranchAuto is finished BAD");
			isFinished = false;
		}
    }
    @Override
    public void end(boolean interrupted){
        if (pathCommand != null) {
			pathCommand.end(interrupted);
		}
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
