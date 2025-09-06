package frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.slamElevator.GenericSlamElevator;
import frc.robot.utils.LoggableTunedNumber;

public class Climber extends GenericSlamElevator<Climber.Goal> {

    public enum Goal implements GenericSlamElevator.SlamElevatorGoal {
        STOP(new LoggableTunedNumber("Climber/StopCurrent", 0.0,Constants.TuningConstants.isTuningClimber), false, SlamElevatorState.IDLING),
        IDLE(new LoggableTunedNumber("Climber/IdleCurrent", -12.0,Constants.TuningConstants.isTuningClimber), true, SlamElevatorState.RETRACTING),
        RETRACT(
                new LoggableTunedNumber("Climber/RetractingCurrent", -40.0,Constants.TuningConstants.isTuningClimber),
                false,
                SlamElevatorState.RETRACTING),
        EXTEND(
                new LoggableTunedNumber("Climber/ExtendingCurrent", 12.0,Constants.TuningConstants.isTuningClimber),
                true,
                SlamElevatorState.EXTENDING);

        private final DoubleSupplier slammingCurrent;
        private final boolean stopAtGoal;
        private final SlamElevatorState state;

        Goal(LoggableTunedNumber slammingCurrent, boolean stopAtGoal, SlamElevatorState state) {
            this.slammingCurrent = slammingCurrent::get;
            this.stopAtGoal = stopAtGoal;
            this.state = state;
        }

        @Override
        public DoubleSupplier getSlammingCurrent() {
            return slammingCurrent;
        }

        @Override
        public boolean isStopAtGoal() {
            return stopAtGoal;
        }

        @Override
        public SlamElevatorState getState() {
            return state;
        }
    }

    private Goal goal = Goal.IDLE;
    public Goal getGoal() {
        return goal;
    }
    public Climber(ClimberIO io) {
        super("Climber", io, 0.4, 1.5);
    }

    @Override
    /**
     * A command which sets to idle, ejects, and then sets to idle again.
     */
    protected Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> goal = Goal.STOP),
                Commands.run(() -> goal = Goal.EXTEND).withTimeout(5),
                Commands.runOnce(() -> {
                    if (!extended()) {
                        addFault(
                                "[System Check] " + getName() + " failed to extend fully",
                                false, true);
                    }
                }),
                //go back down
                Commands.run(() -> goal = Goal.RETRACT).withTimeout(5),
                Commands.runOnce(() -> {
                    if (!retracted()) {
                        addFault(
                                "[System Check] " + getName() + " failed to retract fully",
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.RETRACT));
    }
}