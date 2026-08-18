package frc.robot.subsystems.simpleMechanisms.motor.IntakeFold;

import frc.robot.subsystems.simpleMechanisms.motor.MotorSysIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeFoldIOSim extends MotorSysIOSim implements IntakeFoldIO {
    public static DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static double reduction = 1;
    public static double moi = 1;
    public IntakeFoldIOSim() {
            super(motorModel, reduction, moi);
    }

}
