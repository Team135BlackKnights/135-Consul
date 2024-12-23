package frc.robot.subsystems.state_space.Flywheel;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelIOSpark implements FlywheelIO {
	private double appliedVolts = 0.0;
	private SparkBase flywheel;
	private SparkBaseConfig config;
	private RelativeEncoder encoder;
	private static final Executor CurrentExecutor = Executors
			.newFixedThreadPool(1);

	public FlywheelIOSpark() {
		if (StateSpaceConstants.Flywheel.motorVendor == MotorVendor.NEO_SPARK_MAX) {
			flywheel = new SparkMax(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
			config = new SparkMaxConfig();
		} else {
			flywheel = new SparkFlex(StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
			config = new SparkFlexConfig();
		}
		config.voltageCompensation(12);
		config.idleMode(StateSpaceConstants.Flywheel.isBrake ? IdleMode.kBrake : IdleMode.kCoast);
		flywheel.setCANTimeout(250);
		config.inverted(StateSpaceConstants.Flywheel.inverted);
		config.smartCurrentLimit(StateSpaceConstants.Flywheel.currentLimit);
		encoder = flywheel.getEncoder();
		flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		flywheel.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.velocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.Flywheel.flywheelGearing);
		inputs.currentAmps = new double[] { flywheel.getOutputCurrent()
		};
		inputs.flywheelTemp = flywheel.getMotorTemperature();
	}

	@Override
	public void setVoltage(double volts) {
		appliedVolts = volts;
	}

	@Override
	public void setCurrentLimit(int amps) {
		CurrentExecutor.execute(() -> {
			config.smartCurrentLimit(amps);
			flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		});
	}

	@Override
	/** Stop the flywheel by telling it to go to 0 rpm. */
	public void stop() {
		appliedVolts = 0;
		flywheel.stopMotor();
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Flywheel", flywheel));
		return hardware;
	}
}
