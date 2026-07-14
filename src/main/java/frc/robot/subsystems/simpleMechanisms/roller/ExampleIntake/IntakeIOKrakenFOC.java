package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class IntakeIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IntakeIO {
    public IntakeIOKrakenFOC() {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}
