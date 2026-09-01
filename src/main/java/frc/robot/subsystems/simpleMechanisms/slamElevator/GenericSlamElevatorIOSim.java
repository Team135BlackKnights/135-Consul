package frc.robot.subsystems.simpleMechanisms.slamElevator;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.driverstation.RobotState;
import org.wpilib.simulation.ElevatorSim;

public class GenericSlamElevatorIOSim implements GenericSlamElevatorIO {
  private final ElevatorSim sim;
  private double appliedVoltage = 0.0;
  private final PIDController currentController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);

  private final double drumRadiusMeters;

  /**
   * Creates a new GenericSlamElevator Sim implementation
   *
   * @param maxLengthMeters Position in motor radians from top to bottom of slam elevator
   */
  public GenericSlamElevatorIOSim(
      double maxLengthMeters, double reduction, double drumRadiusMeters) {
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            0.5,
            drumRadiusMeters,
            0.0,
            maxLengthMeters,
            false,
            0.0);
    sim.setState(maxLengthMeters / 2.0, 0);
    this.drumRadiusMeters = drumRadiusMeters;
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    if (!RobotState.isEnabled()) {
      stop();
    }

    sim.update(.02);
    inputs.positionRads = sim.getPosition() / drumRadiusMeters;
    inputs.velocityRadsPerSec = sim.getVelocity();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDraw());
  }

  @Override
  public void runCurrent(double amps) {
    appliedVoltage = currentController.calculate(sim.getCurrentDraw(), amps);
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }
}
