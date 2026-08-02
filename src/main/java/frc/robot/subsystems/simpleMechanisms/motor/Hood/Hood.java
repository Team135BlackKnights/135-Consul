package frc.robot.subsystems.simpleMechanisms.motor.Hood;

import java.util.function.DoubleSupplier;

import com.google.protobuf.Any;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.simpleMechanisms.motor.MotorSys;
import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystem;
import frc.robot.utils.LoggableTunedNumber;

public class Hood extends MotorSys<Hood.Goal> {
     public enum Goal implements MotorSys.PositionGoal {
        STRAIGHT(() -> 0),
        HIGH(new LoggableTunedNumber("HoodAngle/HighPositionRads", 0.3,Constants.TuningConstants.isTuningIntake)),
        LOW(new LoggableTunedNumber("HoodAngle/LowPositionRads", 0.6,Constants.TuningConstants.isTuningIntake));

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

    private Goal goal = Goal.STRAIGHT;

    public Hood(HoodIO io) {
        super("Hood", io);
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
        return null;
    }
}

