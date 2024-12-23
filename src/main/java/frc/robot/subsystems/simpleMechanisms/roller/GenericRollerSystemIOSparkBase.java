package frc.robot.subsystems.simpleMechanisms.roller;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.simpleMechanisms.SimpleMechanismConstants;

/**
 * Generic roller IO implementation for a roller or series of rollers using a
 * SPARK Base.
 */
public abstract class GenericRollerSystemIOSparkBase implements GenericRollerSystemIO {
  private final SparkBase motor;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;
  private final double reduction;
  private final String name;

  public GenericRollerSystemIOSparkBase(
      int id, String name, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    if (SimpleMechanismConstants.Roller.motorType == MotorVendor.NEO_SPARK_MAX) {
      motor = new SparkMax(id, SparkBase.MotorType.kBrushless);
      config = new SparkMaxConfig();
    } else {
      motor = new SparkFlex(id, SparkBase.MotorType.kBrushless);
      config = new SparkFlexConfig();
    }
    this.name = name;
    config = config.smartCurrentLimit(currentLimitAmps).voltageCompensation(12);
    config.inverted(invert);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setCurrentLimit(double amps) {
    config = config.smartCurrentLimit((int) amps);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public List<SelfChecking> getSelfCheckingHardware() {
    List<SelfChecking> hardware = new ArrayList<SelfChecking>();
    hardware.add(new SelfCheckingSparkBase(name, motor));
    return hardware;
  }
}