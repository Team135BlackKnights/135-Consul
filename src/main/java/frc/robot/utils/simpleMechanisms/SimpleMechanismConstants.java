package frc.robot.utils.simpleMechanisms;

import edu.wpi.first.math.util.Units;

public class SimpleMechanismConstants {
    public static class Roller {
        public static final double motorID = 40;
        public static final double conversionFactor = 1.0;

    }

    public static class Climber {
        public static final int id = 41;
        public static final String bus = "";
        public static final String name = "ClimbMotor";
        public static final int currentLimitAmps = 40;
        public static final boolean invert = true;
        public static final double reduction = 60.0 / 1.0;
        public static final double maxLengthMeters = Units.inchesToMeters(15.25);
        public static final double drumRadiusMeters = Units.inchesToMeters(1.275);

    }
    // Arms, either single or double, should ALWAYS use state space models.
}
