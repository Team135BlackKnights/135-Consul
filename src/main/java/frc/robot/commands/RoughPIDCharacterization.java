package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.maths.TimeUtil;

public class RoughPIDCharacterization extends Command {
    private boolean isFinished = false;
    private PIDCharacterizationData data;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<Double> positionSupplier;
    private final Supplier<Double> velocitySupplier;
    private final double minimumPos;
    private final double maximumPos;
    private final double setpointOne;
    private final double setpointTwo;
    private final double timePerTrial;
    private final Timer timer = new Timer();
    private PIDController pidController;
    private double currentP = 0;
    private double currentI = 0;
    private double currentD = 0;
    private boolean isFirstSetpoint = true;
    private boolean forceMoveOn = false;

    public RoughPIDCharacterization(
            Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> positionSupplier,
            Supplier<Double> velocitySupplier, double minimumPos,
            double maximumPos, double setpointOne, double setpointTwo, double timePerTrial, double startingKP) {
        addRequirements(subsystem);
        this.voltageConsumer = voltageConsumer;
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
        this.minimumPos = minimumPos;
        this.maximumPos = maximumPos;
        this.setpointOne = setpointOne;
        this.setpointTwo = setpointTwo;
        this.timePerTrial = timePerTrial;
        this.currentP = startingKP;
        this.pidController = new PIDController(currentP, currentI, currentD);
    }

    @Override
    public void initialize() {
        isFinished = false;
        data = new PIDCharacterizationData(currentP, currentI, currentD, setpointOne);
        timer.start();
    }

    private void getNewController() {
        pidController = data.calculateNewController();
        currentP = pidController.getP();
        currentI = pidController.getI();
        currentD = pidController.getD();
        System.out.println("New Controller: " + currentP + ", " + currentI + ", " + currentD);
        isFirstSetpoint = !isFirstSetpoint; // Toggle between setpoints
        data.reset(currentP, currentI, currentD, isFirstSetpoint ? setpointOne : setpointTwo);
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // all code here
        double elapsedTime = timer.get();
        if (elapsedTime < timePerTrial || forceMoveOn) {
            // Set the PID controller to the first setpoint
            double setpoint = isFirstSetpoint ? setpointOne : setpointTwo;
            pidController.setSetpoint(setpoint);
            voltageConsumer.accept(pidController.calculate(positionSupplier.get()));
            data.add(positionSupplier.get(), velocitySupplier.get());
            if (positionSupplier.get() < minimumPos - .25 || positionSupplier.get() > maximumPos) {
                voltageConsumer.accept(0.0); // Stop the motor if out of bounds
                forceMoveOn = true;
            }
        } else {
            getNewController();
            forceMoveOn = false; // Reset the force move flag after switching setpoints
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(0.0);
        System.out.println("DONE" + interrupted);
        timer.stop();

        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public static class PIDCharacterizationData {
        private double setpoint = 0.0;
        // 2d List of ((position, velocity, time),repeat)
        private final List<List<Double>> data = new LinkedList<>();
        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;
        private double kdCap = -1;
        private double kdStep = 0.01;
        private int kdOscillationCount = 0;

        public PIDCharacterizationData(double kP, double kI, double kD, double setpoint) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.setpoint = setpoint;
        }

        public void add(double position, double velocity) {
            List<Double> dataPoint = new LinkedList<>();
            dataPoint.add(position);
            dataPoint.add(Math.abs(velocity));
            dataPoint.add(TimeUtil.getLogTimeSeconds());
            data.add(dataPoint);

        }

        private void reset(double kP, double kI, double kD, double setpoint) {
            this.setpoint = setpoint;
            data.clear();
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public PIDController calculateNewController() {
            if (kI == 0 && kD == 0) {
                // We're still tuning kP only

                // Detect zero crossings to approximate oscillation
                List<Double> times = new LinkedList<>();
                List<Double> positions = new LinkedList<>();

                for (List<Double> point : data) {
                    positions.add(point.get(0));
                    times.add(point.get(2));
                }

                // Simple zero-crossing detection around the setpoint
                List<Double> zeroCrossTimes = new LinkedList<>();
                for (int i = 1; i < positions.size(); i++) {
                    double prevDiff = positions.get(i - 1) - setpoint;
                    double currDiff = positions.get(i) - setpoint;
                    if ((prevDiff < 0 && currDiff > 0) || (prevDiff > 0 && currDiff < 0)) {
                        zeroCrossTimes.add(times.get(i));
                    }
                }

                double trialDuration = times.isEmpty() ? 5.0 : (times.get(times.size() - 1) - times.get(0));
                int requiredCrossings = (int) (trialDuration / 0.5) * 2;

                if (zeroCrossTimes.size() >= requiredCrossings) {
                    // Calculate average oscillation period (Pc)
                    List<Double> periods = new LinkedList<>();
                    for (int i = 1; i < zeroCrossTimes.size(); i++) {
                        periods.add(zeroCrossTimes.get(i) - zeroCrossTimes.get(i - 1));
                    }

                    double avgPeriod = periods.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
                    double Kc = kP;

                    // Ziegler-Nichols tuning
                    double KP = 0.8 * Kc;
                    double KD = 0.1 * KP * avgPeriod;

                    System.out.println("Oscillation detected. Kc: " + Kc + ", Pc: " + avgPeriod);
                    System.out.println("New PID: KP=" + KP + ", KI=" + 0 + ", KD=" + KD);

                    return new PIDController(KP, 0, KD);
                } else {
                    // Not oscillating yet, increase kP slightly
                    double newKP = kP * 1.1;
                    System.out.println("No oscillation, increasing kP: " + kP + " -> " + newKP);
                    return new PIDController(newKP, 0, 0);
                }
            } else {
                // Post-Ziegler-Nichols tuning with smarter KD capping and no KI
                double peak = Double.NEGATIVE_INFINITY;
                double finalPositionSum = 0.0;
                int finalPoints = 0;
                double startTime = data.get(0).get(2);
                double endTime = data.get(data.size() - 1).get(2);

                List<Double> times = new LinkedList<>();
                List<Double> positions = new LinkedList<>();
                for (List<Double> point : data) {
                    double pos = point.get(0);
                    double time = point.get(2);
                    positions.add(pos);
                    times.add(time);

                    peak = Math.max(peak, pos);
                    if ((time - startTime) >= 0.8 * (endTime - startTime)) {
                        finalPositionSum += pos;
                        finalPoints++;
                    }
                }

                double trialDuration = endTime - startTime;
                double avgFinalPosition = finalPoints == 0 ? setpoint : finalPositionSum / finalPoints;
                double overshoot = peak - setpoint;
                double riseTimeThreshold = 0.9 * setpoint;
                double timeTo90 = -1;

                for (List<Double> point : data) {
                    double pos = point.get(0);
                    double time = point.get(2);
                    if (Math.abs(pos) >= riseTimeThreshold) {
                        timeTo90 = time - startTime;
                        break;
                    }
                }

                // Zero crossing detection → Oscillation = too twitchy
                List<Double> zeroCrossTimes = new LinkedList<>();
                for (int i = 1; i < positions.size(); i++) {
                    double prevDiff = positions.get(i - 1) - setpoint;
                    double currDiff = positions.get(i) - setpoint;
                    if ((prevDiff < 0 && currDiff > 0) || (prevDiff > 0 && currDiff < 0)) {
                        zeroCrossTimes.add(times.get(i));
                    }
                }

                boolean oscillating = zeroCrossTimes.size() > (int) (trialDuration / 0.2);
                if (oscillating) {
                    kdOscillationCount++;
                    if (kdOscillationCount == 1) {
                        kdCap = kD;
                        System.out.println("Oscillation detected. Setting kD cap to " + kdCap);
                    } else {
                        kD *= 0.8;
                        kdStep *= 0.75;
                        System.out.println("Repeated oscillation. Reducing kD and kdStep: " + kD + ", step: " + kdStep);
                    }
                } else {
                    kdOscillationCount = 0;
                    kD += kdStep;
                }

                // Refine kP based on behavior
                if (overshoot > 0.05 * Math.abs(setpoint)) {
                    kP *= 0.95;
                    kD *= 1.05;
                    System.out.println("Overshoot detected. Reducing kP and slightly increasing kD");
                }

                if (timeTo90 > 0.5 * trialDuration) {
                    kP *= 1.05;
                    System.out.println("Rise time slow. Increasing kP: " + kP);
                }

                if (Math.abs(avgFinalPosition - setpoint) < 0.01 * Math.abs(setpoint)) {
                    kP *= 1.02;
                    System.out.println("Near-zero steady-state error. Nudging up kP for speed.");
                }

                // Respect kD cap if defined
                if (kdCap > 0 && kD > kdCap) {
                    kD = kdCap;
                    System.out.println("Capping kD to stable max: " + kD);
                }

                System.out.printf("Refined PID -> kP: %.4f, kD: %.4f%n", kP, kD);
                return new PIDController(kP, 0.0, kD);

            }
        }
    }
}
