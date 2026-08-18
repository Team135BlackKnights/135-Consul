package frc.robot.subsystems.simpleMechanisms.roller.shooter;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim extends GenericRollerSystemIOSim implements shooterIO {
    public static DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static double reduction = 1;
    public static double moi = 1;
    public ShooterIOSim() {
        super(motorModel, reduction, moi);
    }

}
