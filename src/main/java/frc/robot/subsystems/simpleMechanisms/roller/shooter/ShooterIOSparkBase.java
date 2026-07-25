package frc.robot.subsystems.simpleMechanisms.roller.shooter;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSparkBase;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class ShooterIOSparkBase extends GenericRollerSystemIOSparkBase implements shooterIO {
    public ShooterIOSparkBase() {
        super(motorID, name, currentLimitAmps, invert, brake, reduction);
    }
}