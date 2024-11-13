package frc.robot.subsystems.state_space.SingleJointedArm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.state_space.SingleJointedArm.Encoder.SingleJointedArmEncoderIO;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.Sensors.EncoderIO;
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
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

public class SingleJointedArmS extends SubsystemChecker {
	private final SingleJointedArmIO singleJointedArmIO;
	private final EncoderIO singleJointedArmEncoderIO;
	private final EncoderIOInputsAutoLogged singleJointedArmEncoderIOInputs = new EncoderIOInputsAutoLogged();
	private final SingleJointedArmIOInputsAutoLogged inputs = new SingleJointedArmIOInputsAutoLogged();
	private final SysIdRoutine sysId;
	// using sysId
	/*
	 * All SingleJointedArm Statespace uses an N2 at the first position, because we
	 * care about velocity AND position of the SingleJointedarm.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	public static final LinearSystem<N2, N1, N2> m_SingleJointedArmPlant = LinearSystemId
			.createSingleJointedArmSystem(DCMotor.getNEO(1),
					SingleJointedArmSim.estimateMOI(
							StateSpaceConstants.SingleJointedArm.armLength,
							StateSpaceConstants.SingleJointedArm.armMass),
					StateSpaceConstants.SingleJointedArm.armGearing);
	// private final LinearSystem<N2, N1, N1> m_SingleJointedArmPlant =
	// LinearSystemId
	// .identifyPositionSystem(CTRESpaceConstants.SingleJointedArm.armValueHolder.getKv(),
	// CTRESpaceConstants.SingleJointedArm.armValueHolder.getKa());
	private final KalmanFilter<N2, N1, N2> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N2(), m_SingleJointedArmPlant,
			VecBuilder.fill(
					StateSpaceConstants.SingleJointedArm.m_KalmanModelPosition,
					StateSpaceConstants.SingleJointedArm.m_KalmanModelVelocity),
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_KalmanEncoderPosition,
					StateSpaceConstants.SingleJointedArm.m_KalmanEncoderVelocity),
			.02);
	private final LinearQuadraticRegulator<N2, N1, N2> m_controller = new LinearQuadraticRegulator<>(
			m_SingleJointedArmPlant,
			VecBuilder.fill(
					StateSpaceConstants.SingleJointedArm.m_LQRQelmsPosition,
					StateSpaceConstants.SingleJointedArm.m_LQRQelmsVelocity),
			VecBuilder.fill(StateSpaceConstants.SingleJointedArm.m_LQRRVolts),
			.02);
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant
	// for easy control.
	private final LinearSystemLoop<N2, N1, N2> m_loop = new LinearSystemLoop<>(
			m_SingleJointedArmPlant, m_controller, m_observer, 12.0, .02);
	private static double m_velocity, m_position;
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our
	 * SingleJointedarm, allowing smooth motion control for the SingleJointedarm.
	 * Created with constraints based on the motor's free speed, but this will
	 * vary for every system, try tuning these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(
					StateSpaceConstants.SingleJointedArm.maxSpeed, // placeholder
					StateSpaceConstants.SingleJointedArm.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	// set our starting position for the SingleJointedarm
	/*
	 * TrapezoidProfile States are basically just a position in rads with a velocity
	 * in Rad/s
	 * Here, we provide our starting position.
	 */
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.SingleJointedArm.startingPosition, 0);
	Velocity<VoltageUnit> rampRate = Volts.of(1).per(Seconds); // for going FROM ZERO PER SECOND
	Voltage holdVoltage = Volts.of(4);
	Time timeout = Seconds.of(10);
	// Create a Mechanism2d display of an SingleJointedArm with a fixed
	// SingleJointedArmTower and moving SingleJointedArm.
	/*
	 * Mechanism2d is really just an output of the robot, used to debug
	 * SingleJointedarm movement in simulation.
	 * the Mech2d itself is the "canvas" the mechanisms (like SingleJointedarms) are
	 * put on. It will always be your chassis.
	 * A Root2d is the point at which the mechanism rotates, or starts at. Elevators
	 * are different, but here we
	 * simply get it's X and Y according to the robot, then make that the root.
	 * Finally, we make the Ligament itself, and append this to the root point,
	 * basically putting an object at
	 * an origin point RELATIVE to the robot
	 * It takes the current position of the SingleJointedarm, and is the only thing
	 * updated constantly because of that
	 */
	private final Mechanism2d m_mech2d = new Mechanism2d(
			DriveConstants.kChassisWidth, DriveConstants.kChassisLength);
	private final MechanismRoot2d m_SingleJointedarmPivot = m_mech2d.getRoot(
			"SingleJointedArmPivot",
			StateSpaceConstants.SingleJointedArm.physicalX,
			StateSpaceConstants.SingleJointedArm.physicalY);
	private final MechanismLigament2d m_SingleJointedarm = m_SingleJointedarmPivot
			.append(new MechanismLigament2d("SingleJointedArm",
					StateSpaceConstants.SingleJointedArm.armLength,
					Units.radiansToDegrees(inputs.positionRad), 1,
					new Color8Bit(Color.kYellow)));

	public SingleJointedArmS(SingleJointedArmIO io, SingleJointedArmEncoderIO encoderIO) {
		this.singleJointedArmIO = io;
		this.singleJointedArmEncoderIO = encoderIO;
		if (encoderIO != null){
			this.singleJointedArmEncoderIO.setGearRatio(StateSpaceConstants.SingleJointedArm.armGearing);
		}
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
						(state) -> Logger.recordOutput("SingleJointedArmS/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)),
						null, this));
		registerSelfCheckHardware();
		m_loop.reset(VecBuilder.fill(m_position, m_velocity));
		m_lastProfiledReference = new TrapezoidProfile.State(m_position,
				m_velocity);
	}

	@Override
	public void periodic() {
		m_position = inputs.positionRad;
		m_velocity = inputs.velocityRadPerSec;
		m_lastProfiledReference = m_profile.calculate(.02,
				m_lastProfiledReference, goal); // calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); // Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data
		m_loop.correct(VecBuilder.fill(m_position,m_velocity));
		// Update our LQR to generate new voltage commands and use the voltages to
		// predict the next
		// state with out Kalman filter.
		m_loop.predict(.02);
		// Send the new calculated voltage to the motors.
		double appliedVolts = MathUtil.clamp(m_loop.getU(0), -12, 12);
		singleJointedArmIO.setVoltage(appliedVolts);
		singleJointedArmIO.updateInputs(inputs);
		Logger.processInputs("SingleJointedArmS", inputs);
		m_SingleJointedarm.setAngle(Units.radiansToDegrees(m_loop.getXHat(0)));
		Logger.recordOutput("SingleJointedArmMechanism", m_mech2d);
		// calcualate SingleJointedarm pose
		var SingleJointedarmPose = new Pose3d(
				StateSpaceConstants.SingleJointedArm.simX,
				StateSpaceConstants.SingleJointedArm.simY,
				StateSpaceConstants.SingleJointedArm.simZ,
				new Rotation3d(0, -m_loop.getXHat(0), 0.0));
		Logger.recordOutput("Mechanism3d/SingleJointedArm/",
				SingleJointedarmPose);
		if (singleJointedArmEncoderIO != null){
			inputs.positionRad = singleJointedArmEncoderIOInputs.absolutePositionRadians;
			inputs.velocityRadPerSec = singleJointedArmEncoderIOInputs.angularVelocityRadPerSec;
			Logger.processInputs("SingleJointedArmS", inputs);
			Logger.processInputs("SingleJointedArmS/ArmEncoder", singleJointedArmEncoderIOInputs);
		}
	}

	/** Run open loop at the specified voltage. */
	public void runVolts(double volts) {
		singleJointedArmIO.setVoltage(volts);
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.SingleJointedArm.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.SingleJointedArm.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in SingleJointedArmS, called
	 * everywhere else.
	 * 
	 * @param angle IN RADS
	 * @return state, pass through to deployArm.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(angle, 0);
	}

	// Also overload the function to accept both angle in RADS and rad/s
	/**
	 * @param position in RADIANS
	 * @param velocity in RADIANS/SECOND
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
		goal = limitState(state);
		// Log arm setpoint
		Logger.recordOutput("SingleJointedArmS/SetStatePosition", state.position);
		Logger.recordOutput("SingleJointedArmS/SetStateVelocity", state.velocity);
	}

	/**
	 * @return error in radians
	 */
	public double getError() {
		return Math.abs(inputs.positionRad - m_loop.getNextR().get(0, 0));
	}

	public TrapezoidProfile.State limitState(TrapezoidProfile.State state) {
		if (state.position < startingState().position) {
			return startingState();
		} else if (state.position > maxState().position) {
			return maxState();
		}
		return state;
	}

	/** Stops the arm. */
	public void stop() {
		singleJointedArmIO.stop();
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

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction).onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction).onlyWhile(withinLimits(direction));
	}

	/**
	 * Given a direction, is the SingleJointedarm currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < StateSpaceConstants.SingleJointedArm.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		// second set of conditionals (below) checks to see if the SingleJointedarm is
		// within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > StateSpaceConstants.SingleJointedArm.maxPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		// otherwise, we're in bounds!
		returnVal = () -> true;
		return returnVal;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(singleJointedArmIO.getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = singleJointedArmIO.getSelfCheckingHardware();
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
		tempMap.put("SingleArmTemp", inputs.armTemp);
		return tempMap;
	}

	@Override
	public double getCurrent() {
		return inputs.currentAmps[0];
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> setState(createState(Units.degreesToRadians(45))))
						.withTimeout(2.0),
				runOnce(() -> {
					if (getError() > Units.degreesToRadians(5)) {
						addFault(
								"[System Check] Arm angle off more than 5 degrees. Set 45, got "
										+ Units.radiansToDegrees(getDistance()),
								false, true);
					}
				}), run(() -> setState(createState(Units.degreesToRadians(0))))
						.withTimeout(2.0),
				runOnce(() -> {
					if (getError() > Units.degreesToRadians(5)) {
						addFault(
								"[System Check] Arm angle off more than 5 degrees. Set 0, got "
										+ Units.radiansToDegrees(getDistance()),
								false, true);
					}
				})).until(() -> !getFaults().isEmpty());
	}

	@Override
	public void setCurrentLimit(int amps) {
		singleJointedArmIO.setCurrentLimit(amps);
	}
}
