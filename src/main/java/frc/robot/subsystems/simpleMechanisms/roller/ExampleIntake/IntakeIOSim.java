package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
    public IntakeIOSim() {
        super(motorModel, reduction, moi);
    }

}
