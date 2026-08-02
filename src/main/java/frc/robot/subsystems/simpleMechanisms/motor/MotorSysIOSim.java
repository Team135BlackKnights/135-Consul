package frc.robot.subsystems.simpleMechanisms.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorSysIOSim implements MotorSysIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public MotorSysIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel, .1, .1);
  }

  @Override
  public void updateInputs(MotorSysIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setPosition(0.0);
    }

    sim.update(.02);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double angularPositionRads) {
    sim.setAngle(angularPositionRads);
  }

  @Override
  public void stop() {
    setPosition(0.0);
  }
}