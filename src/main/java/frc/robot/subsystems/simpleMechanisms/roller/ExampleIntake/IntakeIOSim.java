package frc.robot.subsystems.simpleMechanisms.roller.ExampleIntake;

import frc.robot.subsystems.simpleMechanisms.roller.GenericRollerSystemIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
    public static DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static double reduction = 1;
    public static double moi = 1;
    public IntakeIOSim() {
        super(motorModel, reduction, moi);
    }

}
