package frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.simpleMechanisms.slamElevator.GenericSlamElevator;
import frc.robot.utils.LoggableTunedNumber;

public class Climber extends GenericSlamElevator<Climber.Goal> {

    public enum Goal implements GenericSlamElevator.SlamElevatorGoal {
        STOP(new LoggableTunedNumber("Climber/StopCurrent", 0.0), false, SlamElevatorState.IDLING),
        IDLE(new LoggableTunedNumber("Climber/IdleCurrent", -12.0), true, SlamElevatorState.RETRACTING),
        RETRACT(
                new LoggableTunedNumber("Climber/RetractingCurrent", -40.0),
                false,
                SlamElevatorState.RETRACTING),
        EXTEND(
                new LoggableTunedNumber("Climber/ExtendingCurrent", 12.0),
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
    public Command systemCheckCommand() {
        return Commands.none();
    }
}