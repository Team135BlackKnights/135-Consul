package frc.robot.utils.servos;

import org.littletonrobotics.junction.AutoLog;


public interface ServoIO {

    @AutoLog
    public static class ServoIOInputs {
        public double PWMValue = 0;
        public double positionPercent = 0;
        public double positionDegrees = 0;
        public double velocityPercent = 0;
        public double velocityDegreesPerSecond = 0;
    }
    public default void updateInputs(ServoIOInputs inputs) {
        
    }
    public default void setDegrees(double degrees) {
        
    }
    public default void setPercent(double percent) {
        
    }
    public default void setVelocity(double percent) {
        
    }
    public default void setVelocityDegrees(double degrees) {
        
    }
}

