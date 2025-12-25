package frc.robot.utils.robotToggles;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class TogglesIONetworkTables implements TogglesIO {
    //simply uses Network Tables instead of physical IO, so no hardware errors are possible
    private final LoggedNetworkBoolean neutralModeSwitch = new LoggedNetworkBoolean("Toggles/NeutralModeSwitch", false);
    private final LoggedNetworkBoolean brakeButton = new LoggedNetworkBoolean("Toggles/HomeButton", false);

    @Override
    public void updateInputs(TogglesIOInputs inputs) {
        neutralModeSwitch.periodic();
        brakeButton.periodic();
        inputs.switchValue = neutralModeSwitch.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        inputs.isHomeButtonPressed = !brakeButton.get();
    }
}
