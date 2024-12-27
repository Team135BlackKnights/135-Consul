package frc.robot.subsystems.state_space.SingleJointedArm;

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

public class SingleJointedArmIOSpark implements SingleJointedArmIO {
	private double appliedVolts = 0.0;
	private SparkBase arm;
	private SparkBaseConfig config;
	private RelativeEncoder encoder;
	private static final Executor CurrentExecutor = Executors
			.newFixedThreadPool(1);

	public SingleJointedArmIOSpark() {
		if (StateSpaceConstants.SingleJointedArm.motorVendor == MotorVendor.NEO_SPARK_MAX) {
			arm = new SparkMax(StateSpaceConstants.SingleJointedArm.kMotorID,
					MotorType.kBrushless);
			config = new SparkMaxConfig();
		} else {
			arm = new SparkFlex(StateSpaceConstants.SingleJointedArm.kMotorID,
					MotorType.kBrushless);
			config = new SparkFlexConfig();
		}
		config.voltageCompensation(12);
		config.idleMode(
				StateSpaceConstants.SingleJointedArm.isBrake ? IdleMode.kBrake
						: IdleMode.kCoast);
		arm.setCANTimeout(250);
		config.inverted(StateSpaceConstants.SingleJointedArm.inverted);
		config.smartCurrentLimit(
				StateSpaceConstants.SingleJointedArm.currentLimit);
		encoder = arm.getEncoder();
		arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	@Override
	public void updateInputs(SingleJointedArmIOInputs inputs) {
		arm.setVoltage(appliedVolts);
		inputs.appliedVolts = appliedVolts;
		inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()
				* StateSpaceConstants.SingleJointedArm.armGearing);
		;
		inputs.armTemp = arm.getMotorTemperature();
		inputs.velocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()
						* StateSpaceConstants.SingleJointedArm.armGearing);
		inputs.currentAmps = new double[] { arm.getOutputCurrent()
		};
	}

	@Override
	public void setVoltage(double volts) {
		arm.setVoltage(volts);
	}

	@Override
	public void setCurrentLimit(int amps) {
		CurrentExecutor.execute(() -> {
			config.smartCurrentLimit(amps);
			arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		});
	}

	@Override
	/** Stop the arm by telling it to go to its same position with 0 speed. */
	public void stop() {
		setVoltage(0);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase("SingleArm", arm));
		return hardware;
	}
}