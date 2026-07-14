package frc.robot.subsystems.simpleMechanisms.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public GenericRollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel, .1, .1);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    sim.update(.02);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}