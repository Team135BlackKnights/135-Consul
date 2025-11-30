package frc.robot.subsystems.advancedMechs.PinkArm.shoulder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO  {
    default void updateInputs(ShoulderIOInputs inputs) {}

    @AutoLog
    class ShoulderIOInputs {
        // Angle is relative to horizontal rest position as 0 position
        public Rotation2d shoulderAngle = Rotation2d.kZero;

        public double shoulderAppliedVolts;
        public double shoulderSupplyCurrentAmps;
        public double shoulderStatorCurrentAmps;
        public double shoulderAngularVelocityRadPerSec;
        public double shoulderAngularAccelerationRadPerSecSquared;

        public double shoulderFRMotorTemp;
        public double shoulderFLMotorTemp;
        public double shoulderBRMotorTemp;
        public double shoulderBLMotorTemp;
    }

    default void setTargetAngle(Rotation2d target) {}

    default void resetShoulderAngle(Rotation2d angle) {}

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
