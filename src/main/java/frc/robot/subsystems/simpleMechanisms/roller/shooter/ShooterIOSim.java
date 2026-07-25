package frc.robot.subsystems.simpleMechanisms.roller.shooter;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class ShooterIOSim extends GenericRollerSystemIOSim implements shooterIO {
    public ShooterIOSim() {
        super(motorModel, reduction, moi);
    }

}
