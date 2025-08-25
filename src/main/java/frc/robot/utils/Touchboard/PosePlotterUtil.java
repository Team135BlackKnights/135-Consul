// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Touchboard;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class PosePlotterUtil {
    // A secondary class containing both a Supplier<Command> and Set<Subsystem> for
    // requirements
    // This is used to create a command from a string, which is then used to create
    // an auto.
    public static class CommandPair {
        public Supplier<Command> command;
        public Set<Subsystem> requirements;

        public CommandPair(Supplier<Command> command, Set<Subsystem> requirements) {
            this.command = command;
            this.requirements = requirements;
        }
    }

    private static NetworkTableInstance inst = NetworkTableInstance.getDefault(); // may cause issues (hasnt so far)
    private static NetworkTable datatable = inst.getTable("touchboard");
    private static HashMap<String, CommandPair> commandPairs = new HashMap<String, CommandPair>();
    private static String defaultAuto = "NA";
    private static StringSubscriber string_Sub = datatable.getStringTopic("posePlotterFinalString").subscribe(defaultAuto);
    private static Command storedAuto = Commands.none();

    public static String getAutoString() {
        String currentString = string_Sub.get();
        return currentString;

    }

    public static void setFallbackAuto(String defaultValue) {
        defaultAuto = defaultValue;
    }

    public static int stringStatus() {
        String currentString = string_Sub.get();
        if (currentString == "NA") {
            return 404;
        } else if (currentString == "unset") {
            return 204;
        } else {
            return 200;

        }
    }

    public static void addCommandPair(String value, CommandPair newCommand) {
        commandPairs.put(value, newCommand);
        return;
    }

    /**
     * Defers the creation of commands until execution time using
     * {@link Commands#defer}.
     * Each command in the autonomous sequence is deferred along with its associated
     * requirements, as specified in {@link CommandPair}.
     *
     * <p>
     * Benefits of deferring command creation:
     * <ul>
     * <li>Allows the autonomous sequence to be constructed independently of the
     * robot's
     * current state, allowing for commands registering to be done before Auto
     * enable.</li>
     * <li>Ensures that commands are instantiated only when they are about to
     * execute,
     * improving memory efficiency.</li>
     * </ul>
     *
     * <p>
     * <strong>Note:</strong> This approach may introduce a small processing
     * overhead
     * during autonomous execution due to deferred instantiation. Specifically, when
     * any given command is called it must call the initialize method within that
     * command.
     * This is a trade-off for the flexibility gained by deferring command creation.
     */
    public static void calculateAuto() {
        System.err.println("Calculating Auto..");
        String startString = PosePlotterUtil.getAutoString();

        System.out.println(startString);
        String[] autoParts = startString.split("_");
        String actions = autoParts[autoParts.length - 1];
        String[] stringArr = actions.split("-");
        Command newAuto = Commands.print("Starting Auto..");
        Command parallelCmd = Commands.none();
        Command nextCommand = Commands.none();
        Boolean currentParallel = false;

        for (String currentValue : stringArr) {

            if (stringArr.length == 0) {
                System.out.println("No Auto!! Consider turning on a fallback if at competition!");
                break;
            }
            nextCommand = Commands.none();

            for (Map.Entry<String, CommandPair> pair : commandPairs.entrySet()) {
                String pairKey = pair.getKey();
                CommandPair pairCommand = pair.getValue();

                if (currentValue.equals(pairKey)) {

                    nextCommand = Commands.defer(pairCommand.command, pairCommand.requirements);
                    break;
                }
            }

            if (nextCommand == Commands.none()) {
                System.out.println(currentValue + " is an undefined command pair!");
            }

            if (currentValue.contains("+")) {
                // If + add to a parallell group with the NEXT command
                parallelCmd = parallelCmd.alongWith(nextCommand);
                currentParallel = true;

                continue;
            } else if (currentParallel) {
                // If previous was + then execute this command with last
                parallelCmd = parallelCmd.alongWith(nextCommand);
                newAuto = newAuto.andThen(parallelCmd);
                parallelCmd = Commands.none();
                currentParallel = false;
            } else {
                // Else Sequence Command
                newAuto = newAuto.andThen(nextCommand);
            }
        }
        storedAuto = newAuto;
    }

    public static Command getAuto() {

        return storedAuto;
    }
}
// HashMap<String, Pose2d> poses = new HashMap<String, Pose2d>();

// poses.put("A", POSES.REEF_A);
// poses.put("B", POSES.REEF_B);
// poses.put("C", POSES.REEF_C);
// poses.put("D", POSES.REEF_D);
// poses.put("E", POSES.REEF_E);
// poses.put("F", POSES.REEF_F);
// poses.put("G", POSES.REEF_G);
// poses.put("H", POSES.REEF_H);
// poses.put("I", POSES.REEF_I);
// poses.put("J", POSES.REEF_J);
// poses.put("K", POSES.REEF_K);
// poses.put("L", POSES.REEF_L);

// poses.put("LT", StationPOSES.Left_top_station);
// poses.put("LM", StationPOSES.Left_mid_station);
// poses.put("LB", StationPOSES.Left_bot_station);
// poses.put("RT", StationPOSES.Right_top_station);
// poses.put("RM", StationPOSES.Right_mid_station);
// poses.put("RB", StationPOSES.Right_bot_station);

// if (DriverStation.getAlliance().isPresent()) {
// if (DriverStation.getAlliance().get() == Alliance.Red) {
// System.out.println("red");
// poses.put("A", FlippingUtil.flipFieldPose(POSES.REEF_A));
// poses.put("B", FlippingUtil.flipFieldPose(POSES.REEF_B));
// poses.put("C", FlippingUtil.flipFieldPose(POSES.REEF_C));
// poses.put("D", FlippingUtil.flipFieldPose(POSES.REEF_D));
// poses.put("E", FlippingUtil.flipFieldPose(POSES.REEF_E));
// poses.put("F", FlippingUtil.flipFieldPose(POSES.REEF_F));
// poses.put("G", FlippingUtil.flipFieldPose(POSES.REEF_G));
// poses.put("H", FlippingUtil.flipFieldPose(POSES.REEF_H));
// poses.put("I", FlippingUtil.flipFieldPose(POSES.REEF_I));
// poses.put("J", FlippingUtil.flipFieldPose(POSES.REEF_J));
// poses.put("K", FlippingUtil.flipFieldPose(POSES.REEF_K));
// poses.put("L", FlippingUtil.flipFieldPose(POSES.REEF_L));

// poses.put("LT", FlippingUtil.flipFieldPose(StationPOSES.Left_top_station));
// poses.put("LM", FlippingUtil.flipFieldPose(StationPOSES.Left_mid_station));
// poses.put("LB", FlippingUtil.flipFieldPose(StationPOSES.Left_bot_station));
// poses.put("RT", FlippingUtil.flipFieldPose(StationPOSES.Right_top_station));
// poses.put("RM", FlippingUtil.flipFieldPose(StationPOSES.Right_mid_station));
// poses.put("RB", FlippingUtil.flipFieldPose(StationPOSES.Right_bot_station));
// }
// }

// PathConstraints constraints = new PathConstraints(
// 3,
// 2,
// 4,
// 3);
// // new PathConstraints(null, null, null, null)

// String startString = posePlotterValues.getAutoString();
// // String startString = posePlotterValues.getAutoStringWithFallback();

// System.out.println(startString);
// String[] stringArr = startString.split("-");
// Command cmd = Commands.none();
// Command parralelCmd = Commands.none();
// Command nextCommand = Commands.none();
// Boolean currentParralel = false;
// for (String a : stringArr) {

// if (a.contains("4S")) {
// nextCommand = new autoshootlfour(-.12, s_ElevatorCom, s_CoralCom,
// false).withTimeout(2);
// } else if (a.contains("4")) {
// nextCommand = new elevatorCom(3, s_ElevatorCom, false);
// } else if (a.contains("0")) {
// nextCommand = new elevatorCom(1, s_ElevatorCom, true);
// } else if (a.matches("[A-L]")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("LT")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("LM")) {
// System.out.println(a);
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("LB")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("RT")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("RM")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// } else if (a.contains("RB")) {
// nextCommand = AutoBuilder.pathfindToPose(
// poses.get(a),
// constraints,
// 0.00);
// }

// else if (stringArr.length == 0) {
// System.out.println("No Command!! Consider turning on fallback if at
// competition!!!!!!!!!!!!!!!");
// } else {
// System.out.println(a + "UNDEFINED COMMAND");
// nextCommand = Commands.none();
// }

// if (a.contains("+")) {
// // cmd = cmd.andThen(Commands.runOnce(()->System.out.println(a +
// // "Simultaneous")));
// parralelCmd = parralelCmd.alongWith(nextCommand);
// currentParralel = true;

// continue;
// } else if (currentParralel) {
// parralelCmd = parralelCmd.alongWith(nextCommand);
// cmd = cmd.andThen(parralelCmd);
// parralelCmd = Commands.none();
// currentParralel = false;
// } else {
// cmd = cmd.andThen(nextCommand);
// }
// }
// ; } else if (a.contains("T")) {
// // cmd = cmd.andThen(Commands.runOnce(() -> System.out.println(a)));
// nextCommand = new Intake(-.07, s_CoralCom);

// } else if (a.contains("S")) {
// nextCommand = (new Shoot(-.12, s_CoralCom));

// } else if (a.contains("3")) {
// nextCommand = new elevatorCom(2, s_ElevatorCom, false);
// } else if (a.contains("2")) {
// nextCommand = new elevatorCom(1, s_ElevatorCom, false);
//