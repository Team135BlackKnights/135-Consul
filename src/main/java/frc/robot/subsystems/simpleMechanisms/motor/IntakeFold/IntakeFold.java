package frc.robot.subsystems.simpleMechanisms.motor.IntakeFold;

import java.util.function.DoubleSupplier;

import com.google.protobuf.Any;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.motor.MotorSys;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;
import frc.robot.utils.LoggableTunedNumber;

public class IntakeFold extends MotorSys<IntakeFold.Goal> {
     public enum Goal implements MotorSys.PositionGoal {
        LIFTED(() -> 0),
        LOWERED(new LoggableTunedNumber("IntakeFold/LowPositionRads", 3.14/4,Constants.TuningConstants.isTuningIntake));

        private final DoubleSupplier positionSupplier;

        Goal(LoggableTunedNumber positionSupplier) {
            this.positionSupplier = positionSupplier::get;
        }

        Goal(DoubleSupplier positionSupplier) {
            this.positionSupplier = positionSupplier;
        }

        @Override
        public DoubleSupplier getPositionSupplier() {
            return positionSupplier;
        }
    }

    private Goal goal = Goal.LIFTED;

    public IntakeFold(IntakeFoldIO io) {
        super("Intake", io);
    }

    public Goal getGoal() {
        return goal;
    }
    
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    @Override
 // not good needs changed
  protected Command systemCheckCommand() {
        return new Command(){
            
        };
    }
}


