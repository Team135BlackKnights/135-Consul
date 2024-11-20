package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class IntakeIOSparkBase extends GenericRollerSystemIOSparkBase implements IntakeIO {
    public IntakeIOSparkBase() {
        super(motorID, name, currentLimitAmps, invert, brake, reduction);
    }
}