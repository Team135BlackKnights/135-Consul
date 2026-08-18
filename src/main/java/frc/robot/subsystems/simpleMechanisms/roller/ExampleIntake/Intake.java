package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;

public class Intake extends GenericRollerSystem<Intake.Goal> {
    public enum Goal implements GenericRollerSystem.VoltageGoal {
        IDLING(() -> 0),
        INTAKING(new LoggableTunedNumber("Intake/AmpScoringVoltage", 12.0,Constants.TuningConstants.isTuningIntake));

        private final DoubleSupplier voltageSupplier;

        Goal(LoggableTunedNumber voltageSupplier) {
            this.voltageSupplier = voltageSupplier::get;
        }

        Goal(DoubleSupplier voltageSupplier) {
            this.voltageSupplier = voltageSupplier;
        }

        @Override
        public DoubleSupplier getVoltageSupplier() {
            return voltageSupplier;
        }
    }

    private Goal goal = Goal.IDLING;

    public Intake(IntakeIO io) {
        super("Intake", io);
    }

    public Goal getGoal() {
        return goal;
    }
    
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    @Override
    /**
     * A command which sets to idle, ejects, and then sets to idle again.
     */
    protected Command systemCheckCommand() {
            return new Command(){
                
            };
        }
}
