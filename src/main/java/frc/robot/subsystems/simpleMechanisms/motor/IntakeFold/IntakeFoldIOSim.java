package frc.robot.subsystems.simpleMechanisms.motor.IntakeFold;

import frc.robot.subsystems.simpleMechanisms.motor.MotorSysIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class IntakeFoldIOSim extends MotorSysIOSim implements IntakeFoldIO {
    public IntakeFoldIOSim() {
            super(motorModel, reduction, moi);
    }

}
