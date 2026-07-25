package frc.robot.subsystems.advancedMechs.Turret;

import edu.wpi.first.math.geometry.Rotation2d;

public class Turret {
    public final class TurretMath {
        private static final int turretTeeth = 77;
        private static final int idlerTeeth = 10;
        private static final double enc1GearTeeth = 36;
        private static final double enc2GearTeeth = 34;
        private static final double enc1ToEnc2Ratio = enc1GearTeeth / enc2GearTeeth;
        private static final double gearGreatestCommonDivisor = greatestCommonDivisor(enc1GearTeeth, enc2GearTeeth);
        private static final int enc1CyclesForPeriod = (int) (enc2GearTeeth / gearGreatestCommonDivisor);

        private static final double twoPi = 2.0 * Math.PI;
        private static final double turretRatio = (double) turretTeeth / idlerTeeth;
        private static final double combinedRatio = turretRatio * enc1ToEnc2Ratio;
        private static final double turretPeriod = twoPi * (idlerTeeth / (double) turretTeeth) * enc1CyclesForPeriod;
        private static final double enc1ModSpan = twoPi / turretRatio;
        private static final int maxIterations = (int) Math.ceil(turretPeriod / enc1ModSpan) + 2;

        private TurretMath() {

        }

        public static double turretAngleFromEncoders(Rotation2d enc1Angle, Rotation2d enc2Angle) {
            return turretAngleFromEncoders(enc1Angle, enc2Angle, 1e-4);
        }

        public static double turretAngleFromEncoders(Rotation2d enc1Angle,
                Rotation2d enc2Angle,
                double tolerance) {
            double baseSolution = mod(-enc1Angle.getRadians() / turretRatio, enc1ModSpan);
            for (int k = 0; k < maxIterations; k++) {
                double candidate = baseSolution + k * enc1ModSpan;
                if (candidate > turretPeriod + tolerance) {
                    break;
                }

                double predictedEnc2 = wrapToTwoPi(combinedRatio * candidate);
                double error = Math.abs(wrapToPi(predictedEnc2 - enc2Angle.getRadians()));
                if (error <= tolerance) {
                    return candidate;
                }
            }

            throw new IllegalArgumentException(
                    "No turret angle lol, crashing to prevent wire damage");
        }

        private static double wrapToTwoPi(double angleRad) {
            double wrapped = angleRad % twoPi;
            return wrapped < 0 ? wrapped + twoPi : wrapped;
        }

        private static double wrapToPi(double angleRad) {
            double wrapped = (angleRad + Math.PI) % twoPi;
            if (wrapped < 0) {
                wrapped += twoPi;
            }
            return wrapped - Math.PI;
        }

        private static double mod(double value, double modulus) {
            double result = value % modulus;
            return result < 0 ? result + modulus : result;
        }

        private static double greatestCommonDivisor(double a, double b) {
            final double EPS = 1e-10;
            a = Math.abs(a);
            b = Math.abs(b);
            if (a < EPS)
                return b;
            if (b < EPS)
                return a;
            while (b > EPS) {
                double temp = b;
                b = a % b;
                a = temp;
            }
            return a;
        }
    }
}
