package frc.robot.utils.CompetitionFieldUtils.Simulation;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.SimplifiedHolonomicDriveSimulation;

import java.util.Arrays;
import java.util.function.Supplier;

public class AIRobotInSimulation {
        /*
         * if an opponent robot is not requested to be on field, it queens outside the
         * field for performance
         */
        public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
                        new Pose2d(-6, 0, new Rotation2d()),
                        new Pose2d(-5, 0, new Rotation2d()),
                        new Pose2d(-4, 0, new Rotation2d()),
                        new Pose2d(-3, 0, new Rotation2d()),
                        new Pose2d(-2, 0, new Rotation2d())
        };
        public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
                        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
                        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
                        new Pose2d(1.6, 6, new Rotation2d()),
                        new Pose2d(1.6, 4, new Rotation2d())
        };
        public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[5];

        private static final AbstractDriveTrainSimulation.DriveTrainSimulationProfile AI_ROBOT_PROFILE = new AbstractDriveTrainSimulation.DriveTrainSimulationProfile(
                        3.8, 12, Math.toRadians(540), Math.toRadians(720),
                        55, 0.8, 0.8);
        private static final RobotConfig robotConfig = new RobotConfig(
                        55,
                        8,
                        new ModuleConfig(
                                        Units.inchesToMeters(2),
                                        3.5,
                                        1.2,
                                        DCMotor.getFalcon500(1).withReduction(8.14),
                                        60,
                                        1),
                        new Translation2d[]{
                                new Translation2d(.6 / 2, .6 / 2),
                                new Translation2d(.6 / 2, -.6 / 2),
                                new Translation2d(-.6 / 2, .6 / 2),
                                new Translation2d(-.6 / 2, -.6 / 2)
                        });
        private static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.02),
                        new PIDConstants(7.0, 0.05));

        private static boolean getIsAlliancePartner(Pose2d startingPose) {
                if (Robot.isRed)
                        return startingPose.getX() < FieldConstants.FIELD_WIDTH / 2;
                return startingPose.getX() > FieldConstants.FIELD_WIDTH / 2;
        }

        public static void startOpponentRobotSimulations() {
                try {
                        
                        instances[0] = new AIRobotInSimulation(
                                        PathPlannerPath.fromPathFile("opponent cycle path 0"),
                                        Commands.none(),
                                        PathPlannerPath.fromPathFile("opponent cycle path 0 reversed"),
                                        Commands.none(),
                                        ROBOT_QUEENING_POSITIONS[0],
                                        1, getIsAlliancePartner(ROBOTS_STARTING_POSITIONS[0]));
                        instances[1] = new AIRobotInSimulation(
                                        PathPlannerPath.fromPathFile("opponent cycle path 1"),
                                        doNothing(),
                                        PathPlannerPath.fromPathFile("opponent cycle path 1 reversed"),
                                        Commands.none(),
                                        ROBOT_QUEENING_POSITIONS[1],
                                        2, getIsAlliancePartner(ROBOTS_STARTING_POSITIONS[1]));
                        instances[2] = new AIRobotInSimulation(
                                        PathPlannerPath.fromPathFile("opponent cycle path 2"),
                                        doNothing(),
                                        PathPlannerPath.fromPathFile("opponent cycle path 2 reversed"),
                                        Commands.none(),
                                        ROBOT_QUEENING_POSITIONS[2],
                                        3, getIsAlliancePartner(ROBOTS_STARTING_POSITIONS[2]));
                        instances[3] = new AIRobotInSimulation(
                                                PathPlannerPath.fromPathFile("opponent cycle path 3"),
                                                doNothing(),
                                                PathPlannerPath.fromPathFile("opponent cycle path 3 reversed"),
                                                Commands.none(),
                                                ROBOT_QUEENING_POSITIONS[3],
                                                4, getIsAlliancePartner(ROBOTS_STARTING_POSITIONS[3]));
                        instances[4] = new AIRobotInSimulation(
                                        PathPlannerPath.fromPathFile("opponent cycle path 4"),
                                        doNothing(),
                                        PathPlannerPath.fromPathFile("opponent cycle path 4 reversed"),
                                        Commands.none(),
                                        ROBOT_QUEENING_POSITIONS[4],
                                        5, getIsAlliancePartner(ROBOTS_STARTING_POSITIONS[4]));
                } catch (Exception e) {
                        DriverStation.reportError(
                                        "failed to load opponent cycle path, error:" + e.getMessage(),
                                        false);
                }
        }

        static private Command doNothing() {
                return Commands.none();
        }

        private final SimplifiedHolonomicDriveSimulation driveSimulation;
        private final int id;

        public AIRobotInSimulation(PathPlannerPath segment0, Command toRunAtEndOfSegment0, PathPlannerPath segment1,
                        Command toRunAtEndOfSegment1, Pose2d queeningPose, int id, boolean isAlliancePartner) {
                this.id = id;
                this.driveSimulation = new SimplifiedHolonomicDriveSimulation(AI_ROBOT_PROFILE, queeningPose, id);
                SendableChooser<Command> behaviorChooser = new SendableChooser<>();
                behaviorChooser.setDefaultOption(
                                "None",
                                Commands.run(() -> driveSimulation.setSimulationWorldPose(queeningPose),
                                                driveSimulation));
                behaviorChooser.addOption(
                                "Auto Cycle",
                                getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));
                behaviorChooser.addOption(
                                "Joystick Drive",
                                getJoystickDriveCommand());
                behaviorChooser.onChange(selected -> {
                if (selected != null) CommandScheduler.getInstance().schedule(selected);
                });
                if (!isAlliancePartner) {
                        RobotModeTriggers.autonomous()
                                        .onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(behaviorChooser.getSelected())));
                }
                RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().schedule(behaviorChooser.getSelected())));
                RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
                        driveSimulation.setSimulationWorldPose(queeningPose);
                        driveSimulation.runChassisSpeeds(new ChassisSpeeds(), false);
                        System.out.println("disabled!!!");
                }, driveSimulation).ignoringDisable(true));

                SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
                RobotContainer.fieldSimulation.addRobot(driveSimulation);

        }

        private Command getAutoCycleCommand(PathPlannerPath segment0, Command toRunAtEndOfSegment0,
                        PathPlannerPath segment1, Command toRunAtEndOfSegment1) {
                final SequentialCommandGroup cycle = new SequentialCommandGroup();
                final Pose2d startingPose = new Pose2d(
                                segment0.getStartingDifferentialPose().getTranslation(),
                                segment0.getIdealStartingState().rotation());
                cycle.addCommands(opponentRobotFollowPath(segment0)
                                .andThen(toRunAtEndOfSegment0)
                                .withTimeout(10));
                cycle.addCommands(opponentRobotFollowPath(segment1)
                                .andThen(toRunAtEndOfSegment1)
                                .withTimeout(10));

                return cycle.repeatedly()
                                .beforeStarting(
                                                Commands.runOnce(() -> driveSimulation
                                                                .setSimulationWorldPose(
                                                                                toCurrentAlliancePose(startingPose))));
        }

        private Command opponentRobotFollowPath(PathPlannerPath path) {
                return new FollowPathCommand(
                                path,
                                driveSimulation::getSimulatedDriveTrainPose,
                                driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                                (speeds, feedForwards) -> driveSimulation.runChassisSpeeds(speeds, false),
                                driveController,
                                robotConfig,
                                () -> Robot.isRed,
                                driveSimulation);
        }

        private Command getJoystickDriveCommand() {
                final XboxController joystick = new XboxController(id);
                final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
                                -joystick.getLeftY() * 3.5,
                                -joystick.getLeftX() * 3.5,
                                -joystick.getRightX() * Math.toRadians(360));
                final Supplier<Rotation2d> opponentDriverStationFacing;
                if (Robot.isRed){
                        if (id == 4 || id == 5){
                                opponentDriverStationFacing = () -> new Rotation2d(Math.PI);
                        }else{
                                opponentDriverStationFacing = () -> new Rotation2d();
                        }
                }else{
                        if (id == 4 || id == 5){
                                opponentDriverStationFacing = () -> new Rotation2d();
                        }else{
                                opponentDriverStationFacing = () -> new Rotation2d(Math.PI);
                        }
                }
                return Commands.run(() -> {
                        driveSimulation.runChassisSpeeds(
                                joystickSpeeds.get()
                                        ,
                                        opponentDriverStationFacing.get());
                        System.out.println("joystick speeds: " + joystick.getLeftY());
                        System.out.println("id: " + id);
                }, driveSimulation)
                                .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                                                toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
        }

        public static Pose2d[] getOpponentRobotPoses() {
                return getRobotPoses(new AIRobotInSimulation[] {
                                instances[0], instances[1], instances[2]
                });
        }

        public static Pose2d[] getAlliancePartnerRobotPoses() {
                return getRobotPoses(new AIRobotInSimulation[] {
                                instances[3], instances[4]
                });
        }

        public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
                final Rotation2d yAxis = Rotation2d.fromDegrees(90),
                                differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis),
                                differenceFromYAxisNew = differenceFromYAxisAtBlueSide
                                                .times(Robot.isRed ? -1 : 1);
                return yAxis.rotateBy(differenceFromYAxisNew);
        }

        public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
                if (Robot.isRed)
                        return new Translation2d(
                                        FieldConstants.FIELD_WIDTH - translationAtBlueSide.getX(),
                                        translationAtBlueSide.getY());
                return translationAtBlueSide;
        }

        public static Pose2d toCurrentAlliancePose(Pose2d poseOnBlue) {
                return new Pose2d(
                                toCurrentAllianceTranslation(poseOnBlue.getTranslation()),
                                toCurrentAllianceRotation(poseOnBlue.getRotation()));
        }

        private static Pose2d[] getRobotPoses(AIRobotInSimulation[] instances) {
                return Arrays.stream(instances)
                                .map(instance -> instance.driveSimulation.getSimulatedDriveTrainPose())
                                .toArray(Pose2d[]::new);
        }
}