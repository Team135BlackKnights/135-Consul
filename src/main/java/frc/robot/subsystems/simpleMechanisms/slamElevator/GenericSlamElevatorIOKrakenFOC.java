package frc.robot.subsystems.simpleMechanisms.slamElevator;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class GenericSlamElevatorIOKrakenFOC implements GenericSlamElevatorIO {
  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Control
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  // Reduction to final sprocket
  private final double reduction;
  
  // Name of the motor
  private final String name;
  public GenericSlamElevatorIOKrakenFOC(
      int id, String bus, String name, int currentLimitAmps, boolean invert, double reduction) {
    talon = new TalonFX(id, bus);
    this.reduction = reduction;
    this.name = name;
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    positionRotations = talon.getPosition();
    velocityRps = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    talon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(GenericSlamElevatorIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.positionRads =
        Units.rotationsToRadians(positionRotations.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocityRps.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runCurrent(double amps) {
    talon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setCurrentLimit(double amps) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = amps;
    talon.getConfigurator().apply(config);
  }
  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
  @Override
  public List<SelfChecking> getSelfCheckingHardware() {
   List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX(name , talon));
    return hardware;
  }
}