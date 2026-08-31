package frc.robot.utils.robotToggles;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.hardware.discrete.DigitalInput;

public class TogglesIOHardware implements TogglesIO {
    private final DigitalInput neutralModeSwitch;
    private final DigitalInput brakeButton;

    public TogglesIOHardware() {
        neutralModeSwitch = new DigitalInput(0);
        brakeButton = new DigitalInput(1);
    }

    @Override
    public void updateInputs(TogglesIOInputs inputs) {
        inputs.switchValue = neutralModeSwitch.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        inputs.isHomeButtonPressed = !brakeButton.get();
    }
}
