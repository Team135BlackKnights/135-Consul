package frc.robot.subsystems.simpleMechanisms.roller.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;

public class shooter extends GenericRollerSystem<shooter.Goal> {
    public enum Goal implements GenericRollerSystem.VoltageGoal {
        IDLING(() -> 0),
        SHOOTING(new LoggableTunedNumber("Intake/AmpScoringVoltage", 12.0,Constants.TuningConstants.isTuningIntake));

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

    public shooter(shooterIO io) {
        super("Intake", io);
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    /**
     * A command which sets to idle, ejects, and then sets to idle again.
     */
    protected Command systemCheckCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> goal = Goal.IDLING),
                Commands.run(() -> goal = Goal.EJECTING).withTimeout(1),
                Commands.runOnce(() -> {
                    if (Math.abs(getAppliedVolts() - Goal.EJECTING.voltageSupplier.getAsDouble()) < .5) {
                        addFault(
                                "[System Check] Ejecting voltage not reached for subsystem:"
                                        + getName(),
                                false, true);
                    }
                }),
                Commands.runOnce(() -> goal = Goal.IDLING));
    }
}