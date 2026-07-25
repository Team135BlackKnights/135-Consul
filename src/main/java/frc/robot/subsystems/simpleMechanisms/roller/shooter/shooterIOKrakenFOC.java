package frc.robot.subsystems.simpleMechanisms.roller.shooter;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOKrakenFOC;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class shooterIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements shooterIO {
    public shooterIOKrakenFOC() {
        super(motorID, bus, name, currentLimitAmps, invert, brake, reduction);
    }
}
