package frc.robot.subsystems.advancedMechs.PinkArm.extension;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;
import java.util.List;

public interface ExtensionIO {
    default void updateInputs(ExtensionIOInputs inputs) {}

    @AutoLog
    class ExtensionIOInputs {
        public double extensionPositionInMeters;

        public double extensionAppliedVolts;
        public double extensionSupplyCurrentAmps;
        public double extensionStatorCurrentAmps;
        public double extensionVelocityMetersPerSec;
        public double extensionAccelerationMetersPerSecSquared;

        public double extensionOneMotorTemp;
        public double extensionTwoMotorTemp;
    }

    default void setTargetExtension(double positionInMeters) {}

    default void resetExtensionPosition(double positionInMeters) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setBrakeMode(boolean enabled) {
    }
    default void setCurrentLimit(int current){}

    /**
     * Get a list of the SelfChecking interface for all hardware in that
     * implementation
     */
    public default List<SelfChecking> getSelfCheckingHardware() {
        return new ArrayList<SelfChecking>();
    }
}
