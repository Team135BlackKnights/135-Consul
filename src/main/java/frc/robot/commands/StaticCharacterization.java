

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class StaticCharacterization extends Command {


  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;
  private static final LoggableTunedNumber currentRampFactor =
      new LoggableTunedNumber("StaticChar/CurrentRampPerSec", 1.0, TuningConstants.isTuningCharacterization); 
  private static final LoggableTunedNumber minVelocity =
      new LoggableTunedNumber("StaticChar/MinStaticVelocity", 0.01, TuningConstants.isTuningCharacterization);
  public StaticCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.get();
    inputConsumer.accept(currentInput);
    Logger.recordOutput("StaticChar/CurrentAmps", currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput + " amps");
    Logger.recordOutput("StaticChar/NeededAmps", currentInput);
    inputConsumer.accept(0);

  }
}