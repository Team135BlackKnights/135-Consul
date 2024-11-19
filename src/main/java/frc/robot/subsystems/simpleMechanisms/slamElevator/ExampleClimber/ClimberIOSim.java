package frc.robot.subsystems.simpleMechanisms.slamElevator.ExampleClimber;

import frc.robot.subsystems.simpleMechanisms.slamElevator.GenericSlamElevatorIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Climber.*;

public class ClimberIOSim extends GenericSlamElevatorIOSim implements ClimberIO {
    public ClimberIOSim() {
        super(maxLengthMeters, reduction, drumRadiusMeters);
    }
}