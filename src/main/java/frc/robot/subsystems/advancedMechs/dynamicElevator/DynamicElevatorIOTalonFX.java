package frc.robot.subsystems.advancedMechs.dynamicElevator;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class DynamicElevatorIOTalonFX implements DynamicElevatorIO {
  // Hardware
  private final TalonFX talon;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  public DynamicElevatorIOTalonFX() {
    talon = new TalonFX(AdvancedMechanismConstants.DynamicElevator.kMotorLeftID, AdvancedMechanismConstants.DynamicElevator.CANBus);
  

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(AdvancedMechanismConstants.DynamicElevator.kP.get())
        .withKI(AdvancedMechanismConstants.DynamicElevator.kI.get()).withKD(AdvancedMechanismConstants.DynamicElevator.kD.get());
    config.Feedback.SensorToMechanismRatio = AdvancedMechanismConstants.DynamicElevator.elevatorGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = AdvancedMechanismConstants.DynamicElevator.currentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -AdvancedMechanismConstants.DynamicElevator.currentLimit;
    config.CurrentLimits.StatorCurrentLimit = AdvancedMechanismConstants.DynamicElevator.currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = AdvancedMechanismConstants.DynamicElevator.leftInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    torqueCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
  

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, torqueCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK())
        break;
    }
  }

  @Override
  public void updateInputs(DynamicElevatorIOInputs inputs) {
    boolean connected = BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)
        .isOK();

    inputs.motorConnected = connectedDebouncer.calculate(connected);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(double positionRad, double velocity, double feedforward) {
    
    talon.setControl(
        positionTorqueCurrentRequest.withVelocity(Units.radiansToRotations(velocity))
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforward * DCMotor.getKrakenX60Foc(1).KtNMPerAmp));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;

    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }
  @Override
  public void setCurrentLimit(int current){
    config.TorqueCurrent.PeakForwardTorqueCurrent = current;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -current;
    config.CurrentLimits.StatorCurrentLimit = current;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }
  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
        () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }

  @Override
  public List<SelfChecking> getSelfCheckingHardware() {
    List<SelfChecking> hardware = new ArrayList<SelfChecking>();
    hardware.add(new SelfCheckingTalonFX("LeftElevator", talon));
    return hardware;
  }
}