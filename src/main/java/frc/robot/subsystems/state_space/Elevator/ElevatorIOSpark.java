package frc.robot.subsystems.state_space.Elevator;

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

public class ElevatorIOSpark implements ElevatorIO {
	private double appliedVolts = 0.0;
	private SparkBase elevator;
	private SparkBaseConfig config;
	private RelativeEncoder encoder;
	private static final Executor CurrentExecutor = Executors
			.newFixedThreadPool(1);

	public ElevatorIOSpark() {
		if (StateSpaceConstants.Elevator.motorVendor == MotorVendor.NEO_SPARK_MAX) {
			elevator = new SparkMax(StateSpaceConstants.Elevator.kMotorID, MotorType.kBrushless);
			config = new SparkMaxConfig();
		} else {
			elevator = new SparkFlex(StateSpaceConstants.Elevator.kMotorID, MotorType.kBrushless);
			config = new SparkFlexConfig();
		}
		config.voltageCompensation(12);
		config.idleMode(StateSpaceConstants.Elevator.isBrake ? IdleMode.kBrake : IdleMode.kCoast);
		elevator.setCANTimeout(250);
		config.inverted(StateSpaceConstants.Elevator.inverted);
		config.smartCurrentLimit(StateSpaceConstants.Elevator.currentLimit);
		encoder = elevator.getEncoder();
		elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		elevator.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionMeters = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.Elevator.elevatorGearing);
		inputs.elevatorTemp = elevator.getMotorTemperature();
		inputs.velocityMetersPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.Elevator.elevatorGearing);
		;
		inputs.currentAmps = new double[] { elevator.getOutputCurrent()
		};
	}

	@Override
	public void setVoltage(double volts) {
		appliedVolts = volts;
	}

	@Override
	public void setCurrentLimit(int amps) {
		CurrentExecutor.execute(() -> {
			config.smartCurrentLimit(amps);
			elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		});
	}

	@Override
	/**
	 * Stop the elevator by telling it to go to its same position with 0 speed.
	 */
	public void stop() {
		setVoltage(0);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("Elevator", elevator));
		return hardware;
	}
}