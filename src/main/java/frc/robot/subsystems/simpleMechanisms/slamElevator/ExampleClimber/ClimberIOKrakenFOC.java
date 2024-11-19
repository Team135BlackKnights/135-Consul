package frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber;

import frc.robot.subsystems.simpleMechanisms.slamElevator.GenericSlamElevatorIOKrakenFOC;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Climber.*;

public class ClimberIOKrakenFOC extends GenericSlamElevatorIOKrakenFOC implements ClimberIO {
    public ClimberIOKrakenFOC() {
        super(id, bus, name, currentLimitAmps, invert, reduction);
    }
}
