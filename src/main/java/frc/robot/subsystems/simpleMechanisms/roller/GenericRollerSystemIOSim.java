package frc.robot.subsystems.simpleMechanisms.roller;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.driverstation.RobotState;
import org.wpilib.simulation.DCMotorSim;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public GenericRollerSystemIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(motorModel, moi, reduction), motorModel, .1,
        .1);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (!RobotState.isEnabled()) {
      runVolts(0.0);
    }

    sim.update(.02);
    inputs.positionRads = sim.getAngularPosition();
    inputs.velocityRadsPerSec = sim.getAngularVelocity();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDraw();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = Math.max(-12.0, Math.min(12.0, volts));
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
