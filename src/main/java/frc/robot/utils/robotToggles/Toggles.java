package frc.robot.utils.robotToggles;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utils.VirtualSubsystem;

//EXTREMELY simple class, just a sample implementation file for robot toggles
//Almost anytime you'd be using RobotToggles, you'll be using a superstructure. In that case, simply within your superstructure class create an instance of TogglesIO and call updateInputs within your superstructure's periodic method
public class Toggles extends VirtualSubsystem{
    private final TogglesIO io;
    private final TogglesIOInputsAutoLogged inputs = new TogglesIOInputsAutoLogged();

    public Toggles(TogglesIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Toggles", inputs);

    }

    public NeutralModeValue getNeutralMode() {
        return inputs.switchValue;
    }

    public boolean isHomeButtonPressed() {
        return inputs.isHomeButtonPressed;
    }

}
