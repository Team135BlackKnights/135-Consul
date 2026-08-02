package frc.robot.subsystems.simpleMechanisms.motor.Hood;

import frc.robot.subsystems.simpleMechanisms.motor.MotorSysIOSim;
import static frc.robot.utils.simpleMechanisms.SimpleMechanismConstants.Roller.*;

public class HoodIOSim extends MotorSysIOSim implements HoodIO {
    public HoodIOSim() {
            super(motorModel, reduction, moi);
    }

}
