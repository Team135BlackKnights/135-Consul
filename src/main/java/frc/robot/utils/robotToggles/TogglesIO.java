package frc.robot.utils.robotToggles;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface TogglesIO {
    default void updateInputs(TogglesIOInputs inputs) {}

    @AutoLog
    class TogglesIOInputs {
        public NeutralModeValue switchValue = NeutralModeValue.Brake;
        public boolean isHomeButtonPressed = false;
    }
}
