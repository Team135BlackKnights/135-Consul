package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.maths.PolynomialRegression;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_VOLTS_PER_SEC = 0.1;

  private FeedForwardCharacterizationData data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;
  private final Supplier<Boolean> atLimit;
  private final Timer timer = new Timer();
  private boolean isFinished = false;
  private String name;
  /** Creates a new FeedForwardCharacterization command. */
  public FeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier, Supplier<Boolean> atLimit) {
    addRequirements(subsystem);
    this.name = subsystem.getName();
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
    this.atLimit = atLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    data = new FeedForwardCharacterizationData(name);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else if (atLimit.get()){
      //end the commmand early
      System.out.println("AT LIMTI");
      isFinished = true;
    } else {
      double voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
      voltageConsumer.accept(voltage);
      data.add(velocitySupplier.get(), voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    System.out.println("DONE" + interrupted);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();
    private final String name;
    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }
    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      System.out.println("FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
      Logger.recordOutput(this.name + "/Characterization/R2", regression.R2());
      Logger.recordOutput(name + "/Characterization/kS", regression.beta(0));
      Logger.recordOutput(name + "/Characterization/kV", regression.beta(1));
    }
  }
}