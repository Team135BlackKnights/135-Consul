package frc.robot.utils.drive;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Utility class for scaling and tuning joystick values from a controller.
 */
public final class TunedJoystick {

    private double deadzone;
    private XboxController cntrllr;
    private Function<Double, Double> responseCurve;
    
    public TunedJoystick(XboxController c) {
        this.cntrllr = c;
        this.responseCurve = ResponseCurve.LINEAR; // Default to linear
        this.deadzone = 0.1d; 
    }
    /* Utility class for predefined response curves */
    public static class ResponseCurve {
        public static final Function<Double, Double> LINEAR = val -> val;
        public static final Function<Double, Double> VERYSOFT = val -> Math.pow(val, 1.48);
        public static final Function<Double, Double> SOFT = val -> Math.pow(val, 1.64);
        public static final Function<Double, Double> QUADRATIC = val -> Math.pow(val, 2.0);
        public static final Function<Double, Double> CUBIC = val -> Math.pow(val, 3.0);
    }


    /* Epic math that scales one domain to a new domain */
    static double map(double val, double in_min, double in_max, double out_min, double out_max) {
        if (val < in_min) {
            return out_min;
        }
        if (val > in_max) {
            return out_max;
        }
        return ((val - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
    }

    /*
     * Returns 0 if input is lower than deadzone.
     * Otherwise, deadzone is the new '0' and scales to the max value (of 1.0)
     * This design implements a square deadzone. A circular deadzone
     * requires the x AND y values to calculate vector magnitude.
     */
    double applyDeadzone(double val) {
        return val > deadzone ? map(val, deadzone, 1.0d, 0.0d, 1.0d) : 0.0d;
    }

/**
     * Set a custom response curve.
     */
    public TunedJoystick useResponseCurve(Function<Double, Double> rc) {
        this.responseCurve = rc;
        return this;
    }

    /**
     * The expected deadzone is between 0.0 and 1.0.
     */
    public TunedJoystick setDeadzone(double d) {
        // Check for negative just in case
        this.deadzone = Math.abs(d);
        return this;
    }

    /*
     * Note that the deadzone must be applied BEFORE the
     * response curve. This is a critical order of operations,
     * so it deserves it's own function for scrutiny.
     * Note that the parameter is an absolute value
     * in order to reduce internal branching.
     */
    double tune(double i) {
        double result = responseCurve.apply(applyDeadzone(Math.abs(i)));
        return (i >= 0.0d) ? result : result * -1.0d;
    }
    /*
     * Tune the input using a custom response curve passed as an argument.
     */
    double tune(double i, Function<Double, Double> customCurve) {
        double result = customCurve.apply(applyDeadzone(Math.abs(i)));
        Logger.recordOutput("Controller/RawVal", i);
        Logger.recordOutput("Controller/Val", result);

        return (i >= 0.0d) ? result : result * -1.0d;
    }
    /**
     * @return Tuned X axis of left joystick.
     */
    public double getLeftX() {
        return tune(cntrllr.getLeftX());
    }

    /**
     * @return Tuned Y axis of left joystick.
     */
    public double getLeftY() {
        return tune(cntrllr.getLeftY());
    }

    /**
     * @return Tuned X axis of right joystick.
     */
    public double getRightX() {
        return tune(cntrllr.getRightX());
    }

    /**
     * @return Tuned Y axis of right joystick.
     */
    public double getRightY() {
        return tune(cntrllr.getRightY());
    }
/**
     * @return Tuned X axis of left joystick.
     */
    public double getLeftX(Function <Double, Double> customCurve) {
        return tune(cntrllr.getLeftX(), customCurve);
    }

    /**
     * @return Tuned Y axis of left joystick.
     */
    public double getLeftY(Function <Double, Double> customCurve) {
        return tune(cntrllr.getLeftY(), customCurve);
    }

    /**
     * @return Tuned X axis of right joystick.
     */
    public double getRightX(Function <Double, Double> customCurve) {
        return tune(cntrllr.getRightX(), customCurve);
    }

    /**
     * @return Tuned Y axis of right joystick.
     */
    public double getRightY(Function <Double, Double> customCurve) {
        return tune(cntrllr.getRightY(), customCurve);
    }
}