package frc.robot.subsystems.advancedMechs.PinkArm.wrist;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    default void updateInputs(WristIOInputs inputs) {}

    @AutoLog
    class WristIOInputs {
        // Angle is relative to starting position
        public Rotation2d wristAngle = Rotation2d.kZero;

        public double wristAppliedVolts;
        public double wristSupplyCurrentAmps;
        public double wristStatorCurrentAmps;
        public double wristAngularVelocityRadPerSec;
        public double wristAngularAccelerationRadPerSecSquared;
        public double wristMotorTemp;
    }

    default void setTargetAngle(Rotation2d target) {}

    default void resetWristAngle(Rotation2d angle) {}

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
    }}
