package frc.robot.subsystems.simpleMechanisms.motor.Hood;

import frc.robot.subsystems.simpleMechanisms.motor.MotorSysIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

import edu.wpi.first.math.system.plant.DCMotor;

public class HoodIOSim extends MotorSysIOSim implements HoodIO {
    public static DCMotor motorModel = DCMotor.getKrakenX60(2);
    public static double reduction = 1;
    public static double moi = 1;
    public HoodIOSim() {
            super(motorModel, reduction, moi);
    }

}
