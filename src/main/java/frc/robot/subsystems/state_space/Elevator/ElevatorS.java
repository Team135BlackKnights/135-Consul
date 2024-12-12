package frc.robot.subsystems.state_space.Elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.EncoderType;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.state_space.Elevator.Encoder.ElevatorEncoderIO;
import frc.robot.utils.drive.Sensors.EncoderIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.state_space.StateSpaceConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ElevatorS extends SubsystemChecker {
	private final ElevatorIO elevatorIO;
	private final ElevatorEncoderIO encoderIO;

	private enum State {
		PERIODIC, CHARACTERIZATION
	}

	private State currentState = State.PERIODIC;
	private final ElevatorIOInputsAutoLogged elevatorIOInputs = new ElevatorIOInputsAutoLogged();
	private final EncoderIOInputsAutoLogged encoderIOInputsAutoLogged = new EncoderIOInputsAutoLogged();
	private static double m_velocity, m_position;
	/*
	 * All Elevator Statespace uses an N2 at the first position, because we care
	 * about velocity AND position of the Elevator.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	public static final LinearSystem<N2, N1, N2> m_elevatorPlant = LinearSystemId
			.createElevatorSystem(DCMotor.getNEO(1),
					StateSpaceConstants.Elevator.carriageMass,
					StateSpaceConstants.Elevator.drumRadius,
					StateSpaceConstants.Elevator.elevatorGearing);
	private final KalmanFilter<N2, N1, N2> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N2(), m_elevatorPlant,
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanModelPosition,
					StateSpaceConstants.Elevator.m_KalmanModelVelocity), // How accurate we
			// think our model is, in meters and meters/second.
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanEncoderPosition,
					StateSpaceConstants.Elevator.m_KalmanEncoderVelocity), // How accurate we think our encoder position
			// data is. In this case we very highly trust our encoder position reading.
			.02);
	private final LinearQuadraticRegulator<N2, N1, N2> m_controller = new LinearQuadraticRegulator<>(
			m_elevatorPlant,
			VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
			// and velocity error tolerances, in meters and meters per second. We care about
			// pos more than velocity.
			VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
			// heavily penalize control effort
			.02);
	private final LinearSystemLoop<N2, N1, N2> m_loop = new LinearSystemLoop<>(
			m_elevatorPlant, m_controller, m_observer, 12.0, .02);
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our
	 * Elevator, allowing smooth motion control for the Elevator. Created with
	 * constraints based on the motor's free speed, but this will vary for every
	 * system, try tuning these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(StateSpaceConstants.Elevator.maxSpeed, // placeholder
					StateSpaceConstants.Elevator.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	// set our starting position for the Elevator
	/*
	 * TrapezoidProfile States are basically just a position in rads with a velocity
	 * in Rad/s
	 * Here, we provide our starting position.
	 */
	private TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Elevator.startingPosition, 0);
	// Create a Mechanism2d display of an Elevator with a fixed ElevatorTower and
	// moving Elevator.
	/*
	 * Mechanism2d is really just an output of the robot, used to debug Elevator
	 * movement in simulation.
	 * the Mech2d itself is the "canvas" the mechanisms (like Elevators) are put on.
	 * It will always be your chassis.
	 * A Root2d is the point at which the mechanism rotates, or starts at. Elevators
	 * are different, but here we
	 * simply get it's X and Y according to the robot, then make that the root.
	 * Finally, we make the Ligament itself, and append this to the root point,
	 * basically putting an object at
	 * an origin point RELATIVE to the robot
	 * It takes the current position of the Elevator, and is the only thing updated
	 * constantly because of that
	 */
	private final LoggedMechanism2d m_mech2d = new LoggedMechanism2d(
			StateSpaceConstants.Elevator.maxPosition + .25,
			StateSpaceConstants.Elevator.maxPosition + .25);
	private final LoggedMechanismRoot2d m_mech2dRoot = m_mech2d.getRoot(
			"Elevator Root", StateSpaceConstants.Elevator.physicalX,
			StateSpaceConstants.Elevator.physicalY);
	private final LoggedMechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
			new LoggedMechanismLigament2d("Elevator", elevatorIOInputs.positionMeters, 90));

	public ElevatorS(ElevatorIO elevatorIO, ElevatorEncoderIO encoderIO) {
		this.elevatorIO = elevatorIO;
		this.encoderIO = encoderIO;
		if (encoderIO != null) {
			this.encoderIO.setGearRatio(StateSpaceConstants.Flywheel.flywheelGearing);
		}
		registerSelfCheckHardware();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
				m_velocity);
	}

	@Override
	public void periodic() {
		long timestamp = System.currentTimeMillis();
		if (encoderIO != null) {
			
			encoderIO.updateInputs(encoderIOInputsAutoLogged);
			if (encoderIOInputsAutoLogged.encoderType != EncoderType.NO_ATTACHED_ENCODER) {
				elevatorIOInputs.positionMeters = encoderIOInputsAutoLogged.absolutePositionRadians;
				elevatorIOInputs.velocityMetersPerSec = encoderIOInputsAutoLogged.angularVelocityRadPerSec;
			}
			Logger.processInputs("ElevatorS/ElevatorEncoder", encoderIOInputsAutoLogged);
			Logger.recordOutput("SystemStatus/Periodic/ElevatorEncoderMS", System.currentTimeMillis() - timestamp);
			timestamp = System.currentTimeMillis();
		}
		m_position = elevatorIOInputs.positionMeters;
		m_velocity = elevatorIOInputs.velocityMetersPerSec;

		m_lastProfiledReference = m_profile.calculate(.02,
				m_lastProfiledReference, goal); // calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); // Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data
		m_loop.correct(VecBuilder.fill(m_position, m_velocity));
		// Update our LQR to generate new voltage commands and use the voltages to
		// predict the next
		// state with out Kalman filter.
		m_loop.predict(.02);
		if (currentState != State.CHARACTERIZATION) {
			// Send the new calculated voltage to the motors.
			double appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
			elevatorIO.setVoltage(appliedVolts);
		}
		m_elevatorMech2d.setLength(m_loop.getXHat(0));
		// Push the mechanism to AdvantageScope
		Logger.recordOutput("ElevatorMechanism", m_mech2d);
		// calcualate arm pose
		var elevatorPose = new Pose3d(StateSpaceConstants.Elevator.simX,
				StateSpaceConstants.Elevator.simY,
				StateSpaceConstants.Elevator.simZ, new Rotation3d(0, 0, 0.0));
		Logger.recordOutput("Mechanism3d/Elevator/", elevatorPose);
		Logger.recordOutput("SystemStatus/Periodic/ElevatorProcessMS", System.currentTimeMillis() - timestamp);
		timestamp = System.currentTimeMillis();
		elevatorIO.updateInputs(elevatorIOInputs);
		Logger.processInputs("ElevatorS", elevatorIOInputs);
		Logger.recordOutput("SystemStatus/Periodic/ElevatorInputsMS", System.currentTimeMillis() - timestamp);
		
	}

	/** Run open loop at the specified voltage. */
	public void runVolts(double volts) {
		currentState = State.CHARACTERIZATION;
		elevatorIO.setVoltage(volts);
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in ElevatorS, called everywhere
	 * else.
	 * 
	 * @param position IN meters
	 * @return state, pass through to setState
	 */
	public TrapezoidProfile.State createState(double position) {
		return new TrapezoidProfile.State(position, 0);
	}

	// Also overload the function to accept both angle in METERS and METER/s
	/**
	 * @param position in METERS
	 * @param velocity in METES/SECOND
	 * @return state, pass through to deployArm.
	 */
	public TrapezoidProfile.State createState(double position, double velocity) {
		return new TrapezoidProfile.State(position, velocity);
	}

	/**
	 * Checks if close to state, with less than specified degrees
	 * 
	 * @param degrees error in degrees
	 * @return true if at state
	 */
	public boolean isAtState(double degrees) {
		if (Math.abs(getError()) < Units.degreesToRadians(degrees)) {
			return true;
		}
		return false;
	}

	/** Run closed loop to the specified state. */
	public void setState(TrapezoidProfile.State state) {
		currentState = State.PERIODIC;
		goal = limitState(state);
		// Log arm setpoint
		Logger.recordOutput("ElevatorS/SetStatePosition", state.position);
		Logger.recordOutput("ElevatorS/SetStateVelocity", state.velocity);
	}

	/**
	 * @return error in meters from RAW encoder
	 */
	public double getError() {
		return Math.abs(m_position - m_loop.getNextR().get(0, 0));
	}

	/** Stops the arm. */
	public void stop() {
		elevatorIO.stop();
	}

	public double getDistance() {
		return m_loop.getXHat(0);
	}

	public double getSetpoint() {
		return goal.position;
	}

	public double getVelocity() {
		return m_loop.getXHat(1);
	}

	public TrapezoidProfile.State limitState(TrapezoidProfile.State state) {
		if (state.position < startingState().position) {
			return startingState();
		} else if (state.position > maxState().position) {
			return maxState();
		}
		return state;
	}

	public double getCharacterizationVelocity() {
		return elevatorIOInputs.velocityMetersPerSec;
	}

	public boolean isCharacterizationInLimit() {
		return withinLimits(SysIdRoutine.Direction.kForward).getAsBoolean();
	}

	public void endCharacterization() {
		elevatorIO.stop();
		currentState = State.PERIODIC;
	}

	/**
	 * Given a direction, is the Elevator currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (elevatorIOInputs.positionMeters <= StateSpaceConstants.Elevator.startingPosition) {
				returnVal = () -> true;
				return returnVal;
			}
		}
		// second set of conditionals (below) checks to see if the Elevator is within
		// the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			System.out.println("Elevator Position: " + elevatorIOInputs.positionMeters);
			if (elevatorIOInputs.positionMeters >= StateSpaceConstants.Elevator.maxPosition) {
				returnVal = () -> true;
				System.out.println("Elevator Position: " + elevatorIOInputs.positionMeters);
				return returnVal;
			}
		}
		// otherwise, we're in bounds!
		returnVal = () -> false;
		return returnVal;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(elevatorIO.getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = elevatorIO.getSelfCheckingHardware();
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
		tempMap.put("ElevatorTemp", elevatorIOInputs.elevatorTemp);
		return tempMap;
	}

	@Override
	public void setCurrentLimit(int amps) {
		elevatorIO.setCurrentLimit(amps);
	}

	@Override
	public double getCurrent() {
		return elevatorIOInputs.currentAmps[0];
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> setState(createState(Units.feetToMeters(2))))
						.withTimeout(2.0), runOnce(() -> {
							if (getError() > Units.inchesToMeters(4)) {
								addFault(
										"[System Check] Elevator position off more than 4 inches. Set 2ft, got "
												+ Units.metersToFeet(getDistance()),
										false, true);
							}
						}), run(() -> setState(createState(Units.feetToMeters(0))))
								.withTimeout(2.0),
						runOnce(() -> {
							if (getError() > Units.inchesToMeters(4)) {
								addFault(
										"[System Check] Elevator position off more than 4 inches. Set 0ft, got "
												+ Units.metersToFeet(getDistance()),
										false, true);
							}
						}))
				.until(() -> !getFaults().isEmpty());
	}
}
