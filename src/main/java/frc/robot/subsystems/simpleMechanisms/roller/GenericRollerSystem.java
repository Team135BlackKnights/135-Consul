package frc.robot.subsystems.simpleMechanisms.roller;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

public abstract class GenericRollerSystem<G extends GenericRollerSystem.VoltageGoal> extends SubsystemChecker {
  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs = new GenericRollerSystemIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.io = io;

    stateTimer.start();
    registerSelfCheckHardware();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }

  public HashMap<String, Double> getTemps() {
    HashMap<String, Double> tempMap = new HashMap<>();
    tempMap.put(inputs.name, inputs.tempCelsius);
    return tempMap;
  }
  public double getAppliedVolts(){
    return inputs.appliedVoltage;
  }
  public String getName(){
    return name;
  }
  private void registerSelfCheckHardware() {
    super.registerAllHardware(io.getSelfCheckingHardware());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    List<ParentDevice> orchestra = new ArrayList<>();
    List<SelfChecking> hardware = io.getSelfCheckingHardware();
    for (SelfChecking motor : hardware) {
      if (motor.getHardware() instanceof TalonFX) {
        orchestra.add((TalonFX) motor.getHardware());
      }
    }
    return orchestra;
  }

  @Override
  public double getCurrent() {
    return inputs.supplyCurrentAmps;
  }

  @Override
  public void setCurrentLimit(int amps) {
    io.setCurrentLimit(amps);
  }
}