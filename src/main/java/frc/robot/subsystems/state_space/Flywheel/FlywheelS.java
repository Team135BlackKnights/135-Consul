package frc.robot.subsystems.state_space.Flywheel;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EncoderType;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.state_space.Flywheel.Encoder.FlywheelEncoderIO;
import frc.robot.utils.drive.Sensors.EncoderIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.state_space.StateSpaceConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class FlywheelS extends SubsystemChecker {
	private final FlywheelIO flywheelIO;
	private final FlywheelEncoderIO encoderIO;
	private final FlywheelIOInputsAutoLogged flywheelIOInputs = new FlywheelIOInputsAutoLogged();
	private final EncoderIOInputsAutoLogged encoderIOInputsAutoLogged = new EncoderIOInputsAutoLogged();
	private enum State {
		PERIODIC, CHARACTERIZATION
	}
	private State currentState = State.PERIODIC;
	/**
	 * This Plant holds a state-space model of our flywheel. It has the following
	 * properties: States: Velocity, in Rad/s. (will match Output) Inputs: Volts.
	 * Outputs: Velcoity, in Rad/s Flywheels require either an MOI or Kv and Ka,
	 * which are found using SysId.
	 */
	public final static LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId
			// .createFlywheelSystem(DCMotor.getNEO(1),StateSpaceConstants.Flywheel.MOI,StateSpaceConstants.Flywheel.flywheelGearing);
			.identifyVelocitySystem(
					StateSpaceConstants.Flywheel.flywheelValueHolder.getKv(),
					StateSpaceConstants.Flywheel.flywheelValueHolder.getKa());
	/**
	 * Kalman filters are optional, and help to predict the actual state of the
	 * system given a noisy encoder (which all encoders are!) Takes two empty
	 * N1's, our plant, and then our model accuracy in St. Devs and our Encoder
	 * value in St. Devs. Higher value means less trust, as an example 1 means we
	 * trust it 68% of all time.
	 */
	private final static KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N1(), Nat.N1(), flywheelPlant,
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanModel),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanEncoder), .02);
	/**
	 * A Linear Quadratic Regulator (LQR) controls linear systems. It minimizes
	 * cost, or voltage, to minimize state error. So, get to a position as fast,
	 * and efficient, as possible Our Qelms, or Q, is our PENALTY for deviation.
	 * Qelms is for our output, here being RPM, and R (Volts) is our penalty for
	 * control effort. Basically, high either of these means that it will take
	 * that more into account. A Qelms value of 100 means it REALLY cares about
	 * our RPM, and penalizes HEAVILY for incorrectness.
	 */
	private final static LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			flywheelPlant,
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRQelms),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRRVolts), .02);
	/**
	 * A state-space loop combines a controller, observer, feedforward, and plant
	 * all into one. It does everything! The random 12 is the upper limit on
	 * Voltage the loop can output. Never require more than 12 V, because.. we
	 * can't give it!
	 */
	private final static LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
			flywheelPlant, m_controller, m_observer, 12, .02);

	/**
	 * Creates a new flywheelS
	 * 
	 * @param flywheelIO the flywheel IO
	 * @param encoderIO  the encoder IO (can leave this one null if no independent
	 *                   encoder attached, flywheel will just use its built in one)
	 */
	public FlywheelS(FlywheelIO flywheelIO, FlywheelEncoderIO encoderIO) {
		this.flywheelIO = flywheelIO;
		this.encoderIO = encoderIO;
		if (encoderIO != null) {
			this.encoderIO.setGearRatio(StateSpaceConstants.Flywheel.flywheelGearing);
		}
		m_loop.setNextR(0); // go to zero
		m_loop.reset(VecBuilder.fill(0));
		registerSelfCheckHardware();
	}

	@Override
	public void periodic() {
		long timestamp = System.currentTimeMillis();
		if (encoderIO != null) {
			encoderIO.updateInputs(encoderIOInputsAutoLogged);
			if (encoderIOInputsAutoLogged.encoderType != EncoderType.NO_ATTACHED_ENCODER) {
				flywheelIOInputs.positionRad = encoderIOInputsAutoLogged.absolutePositionRadians;
				flywheelIOInputs.velocityRadPerSec = encoderIOInputsAutoLogged.angularVelocityRadPerSec;
			}
			flywheelIOInputs.positionRad = encoderIOInputsAutoLogged.absolutePositionRadians;
			flywheelIOInputs.velocityRadPerSec = encoderIOInputsAutoLogged.angularVelocityRadPerSec;
			Logger.processInputs("FlywheelS/FlywheelEncoder", encoderIOInputsAutoLogged);
			Logger.recordOutput("SystemStatus/Periodic/FlywheelEncoderMS", System.currentTimeMillis() - timestamp);
			timestamp = System.currentTimeMillis();
		}
		m_loop.correct(VecBuilder.fill(flywheelIOInputs.velocityRadPerSec));
		m_loop.predict(.02);
		if (currentState != State.CHARACTERIZATION) {
			double volts = MathUtil.clamp(m_loop.getU(0), -12, 12);
			flywheelIO.setVoltage(volts);	
			Logger.recordOutput("FlywheelS/Volts", volts);	
		}
		Logger.recordOutput("FlywheelS/AdjustedRPM",
				Units.radiansPerSecondToRotationsPerMinute(m_loop.getXHat(0)));
		Logger.recordOutput("SystemStatus/Periodic/FlywheelProcessMS", System.currentTimeMillis() - timestamp);
		timestamp = System.currentTimeMillis();
		flywheelIO.updateInputs(flywheelIOInputs);
		Logger.processInputs("FlywheelS", flywheelIOInputs);
		Logger.recordOutput("SystemStatus/Periodic/FlywheelInputsMS", System.currentTimeMillis() - timestamp);
	}

	/** Run open loop at the specified voltage. */
	public void runVolts(double volts) {
		currentState = State.CHARACTERIZATION;
		flywheelIO.setVoltage(volts);
	}

	/** Run closed loop at the specified velocity. */
	public void setRPM(double velocityRPM) {
		currentState = State.PERIODIC;
		var velocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
		m_loop.setNextR(VecBuilder.fill(velocityRadPerSec));
		// Log flywheel setpoint
		Logger.recordOutput("FlywheelS/SetpointRPM", velocityRPM);
	}

	/** Stops the flywheel. */
	public void stop() {
		m_loop.setNextR(VecBuilder.fill(0));
		flywheelIO.stop();
	}
	public void endCharacterization() {
		flywheelIO.stop();
		currentState = State.PERIODIC;
	}
	/** Returns the current velocity in RPM. */
	@AutoLogOutput
	public double getRPM() {
		return Units.radiansPerSecondToRotationsPerMinute(m_loop.getXHat(0));
	}

	public double getError() {
		return Math.abs(flywheelIOInputs.velocityRadPerSec - m_loop.getNextR().get(0, 0));
	}

	@Override
	public double getCurrent() {
		return Math.abs(flywheelIOInputs.currentAmps[0]);
	}

	/** Returns the current velocity in radians per second. */
	public double getCharacterizationVelocity() {
		return flywheelIOInputs.velocityRadPerSec;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(flywheelIO.getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = flywheelIO.getSelfCheckingHardware();
		for (SelfChecking motor : driveHardware) {
			if (motor.getHardware() instanceof TalonFX) {
				orchestra.add((TalonFX) motor.getHardware());
			}
		}
		return orchestra;
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FlywheelTemp", flywheelIOInputs.flywheelTemp);
		return tempMap;
	}

	@Override
	public void setCurrentLimit(int amps) {
		flywheelIO.setCurrentLimit(amps);
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> setRPM(4000)).withTimeout(1.5), runOnce(() -> {
					if (getError() > 50) {
						addFault(
								"[System Check] Flywheel speed off more than 50. Set 4000, got "
										+ getRPM(),
								false, true);
					}
				}), run(() -> setRPM(6000)).withTimeout(1.5), runOnce(() -> {
					if (getError() > 100) {
						addFault(
								"[System Check] Flywheel speed off more than 100. Set 6000, got "
										+ getRPM(),
								false, true);
					}
				})).until(() -> !getFaults().isEmpty())
				.andThen(runOnce(() -> setRPM(0)));
	}
}
