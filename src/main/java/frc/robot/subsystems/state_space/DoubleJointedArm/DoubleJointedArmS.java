package frc.robot.subsystems.state_space.DoubleJointedArm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.EncoderType;
import frc.robot.DataHandler;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.state_space.DoubleJointedArm.ArmEncoder.DoubleJointedArmArmEncoderIO;
import frc.robot.subsystems.state_space.DoubleJointedArm.ElbowEncoder.DoubleJointedArmElbowEncoderIO;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.Sensors.EncoderIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.state_space.StateSpaceConstants;

public class DoubleJointedArmS extends SubsystemChecker {
	private final DoubleJointedArmIO doubleJointedArmIO;
	private final DoubleJointedArmArmEncoderIO armEncoderIO;
	private final DoubleJointedArmElbowEncoderIO elbowEncoderIO;
	private final DoubleJointedArmIOInputsAutoLogged doubleJointedArmInputs = new DoubleJointedArmIOInputsAutoLogged();
	private final EncoderIOInputsAutoLogged armEncoderIOInputsAutoLogged = new EncoderIOInputsAutoLogged();
	private final EncoderIOInputsAutoLogged elbowEncoderIOInputsAutoLogged = new EncoderIOInputsAutoLogged();
	private enum State {
		PERIODIC, CHARACTERIZATION
	}
	private State currentState = State.PERIODIC;
	private List<Double> voltages;
	private double armSetRad;
	private double elbowSetRad;
	public double latency;
	private Notifier m_updatePositionsNotifier = null; // Checks for updates
	private final LoggedMechanism2d  m_mech2d = new LoggedMechanism2d(
			StateSpaceConstants.DoubleJointedArm.simSizeWidth,
			StateSpaceConstants.DoubleJointedArm.simSizeLength);
	private final LoggedMechanismRoot2d m_DoubleJointedarmPivot = m_mech2d.getRoot(
			"DoubleJointedArmPivot",
			StateSpaceConstants.DoubleJointedArm.physicalX,
			StateSpaceConstants.DoubleJointedArm.physicalY);
	private final LoggedMechanismLigament2d m_DoubleJointedArm = m_DoubleJointedarmPivot
			.append(new LoggedMechanismLigament2d("DoubleJointedArm",
					StateSpaceConstants.DoubleJointedArm.armLength,
					Units.radiansToDegrees(getArmRads()), 1,
					new Color8Bit(Color.kYellow)));
	private final LoggedMechanismLigament2d m_DoubleJointedElbow = m_DoubleJointedArm
			.append(new LoggedMechanismLigament2d("DoubleJointedElbow",
					StateSpaceConstants.DoubleJointedArm.elbowLength,
					Units.radiansToDegrees(getElbowRads()), 1,
					new Color8Bit(Color.kYellow)));

	public DoubleJointedArmS(DoubleJointedArmIO io, DoubleJointedArmArmEncoderIO armEncoderIO,
			DoubleJointedArmElbowEncoderIO elbowEncoderIO) {
		this.armEncoderIO = armEncoderIO;
		this.elbowEncoderIO = elbowEncoderIO;
		if (this.armEncoderIO != null) {
			this.armEncoderIO.setGearRatio(StateSpaceConstants.DoubleJointedArm.armGearing);
		}
		if (this.elbowEncoderIO != null) {
			this.elbowEncoderIO.setGearRatio(StateSpaceConstants.DoubleJointedArm.elbowGearing);
		}
		this.doubleJointedArmIO = io;
		m_updatePositionsNotifier = new Notifier(() -> {
			DataHandler.logData(new double[] { getArmRads(), getElbowRads()
			}, "DoubleJointedEncoders");
		});
		m_updatePositionsNotifier.startPeriodic(.02);
		registerSelfCheckHardware();
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(doubleJointedArmIO.getSelfCheckingHardware());
	}

	public void setVoltages(List<Double> voltages) {
		this.voltages = voltages;
	}

	@Override
	public void periodic() {
		long startTime = System.currentTimeMillis();
		if (voltages != null && currentState != State.CHARACTERIZATION) {
			doubleJointedArmIO.setVoltage(voltages);
		}
		if (elbowEncoderIO != null) {
			elbowEncoderIO.updateInputs(elbowEncoderIOInputsAutoLogged);
			if (elbowEncoderIOInputsAutoLogged.encoderType != EncoderType.NO_ATTACHED_ENCODER) {
				doubleJointedArmInputs.positionElbowRads = elbowEncoderIOInputsAutoLogged.absolutePositionRadians;
				doubleJointedArmInputs.velocityElbowRadsPerSec = elbowEncoderIOInputsAutoLogged.angularVelocityRadPerSec;
			}
			Logger.processInputs("DoubleJointedArmS/ElbowEncoder", elbowEncoderIOInputsAutoLogged);
		}
		if (armEncoderIO != null) {
			armEncoderIO.updateInputs(armEncoderIOInputsAutoLogged);
			if (armEncoderIOInputsAutoLogged.encoderType != EncoderType.NO_ATTACHED_ENCODER) {
				doubleJointedArmInputs.positionArmRads = armEncoderIOInputsAutoLogged.absolutePositionRadians;
				doubleJointedArmInputs.velocityArmRadsPerSec = armEncoderIOInputsAutoLogged.angularVelocityRadPerSec;
			}
			Logger.processInputs("DoubleJointedArmS/Periodic/ArmEncoder", armEncoderIOInputsAutoLogged);
		}
		doubleJointedArmIO.updateInputs(doubleJointedArmInputs);
		Logger.processInputs("DoubleJointedArmS", doubleJointedArmInputs);
		Logger.recordOutput("SystemStatus/Periodic/DoubleJointedArmInputsMS", System.currentTimeMillis() - startTime);
		//processing time will ALWAYS be zero, so don't bother logging it.
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			// send the new qelms/relms to the Pi
			double[] currentConstants = new double[] { StateSpaceConstants.DoubleJointedArm.qPos.get(),
					StateSpaceConstants.DoubleJointedArm.qVel.get(), StateSpaceConstants.DoubleJointedArm.qError.get(),
					StateSpaceConstants.DoubleJointedArm.rPos.get() };
			DataHandler.logData(currentConstants, "DoubleJointedArmConstants");
			Logger.recordOutput("DoubleJointedArmS/currentConstants", currentConstants);
		}, StateSpaceConstants.DoubleJointedArm.qPos,
				StateSpaceConstants.DoubleJointedArm.qVel, StateSpaceConstants.DoubleJointedArm.qError,
				StateSpaceConstants.DoubleJointedArm.rPos);
		m_DoubleJointedArm.setAngle(Units.radiansToDegrees(getArmRads()));
		m_DoubleJointedElbow.setAngle(Units.radiansToDegrees(getElbowRads()));
		Logger.recordOutput("DoubleJointedArmS/DoubleJointedArmMechanism", m_mech2d);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = doubleJointedArmIO.getSelfCheckingHardware();
		for (SelfChecking motor : driveHardware) {
			if (motor.getHardware() instanceof TalonFX) {
				orchestra.add((TalonFX) motor.getHardware());
			}
		}
		return orchestra;
	}

	public double getArmError() {
		return doubleJointedArmInputs.positionArmRads - armSetRad;
	}

	public void setExpectedPositions(List<Double> rads) {
		doubleJointedArmIO.setExpectedPositions(rads.get(0), rads.get(1));
	}

	public double getElbowError() {
		return doubleJointedArmInputs.positionElbowRads - elbowSetRad;
	}

	public void setDoubleJointedArm(double armRad, double elbowRad,
			double elbowInversed) {
		this.armSetRad = limitArmRad(armRad);
		this.elbowSetRad = limitElbowRad(elbowRad);
		DataHandler.logData(new double[] { armRad, elbowRad, elbowInversed
		}, "DoubleJointSetpoint");
		Logger.recordOutput("DoubleJointedArmS/setpoint", new double[] { armRad, elbowRad, elbowInversed
		});
	}

	public void setDoubleJointedArm(double[] macro) {
		this.armSetRad = limitArmRad(macro[0]);
		this.elbowSetRad = limitElbowRad(macro[1]);
		DataHandler.logData(macro, "DoubleJointSetpoint");
		Logger.recordOutput("DoubleJointedArmS/setpoint", macro);
	}

	public double limitArmRad(double armRad) {
		if (armRad < StateSpaceConstants.DoubleJointedArm.armMinRad) {
			return StateSpaceConstants.DoubleJointedArm.armMinRad;
		} else if (armRad > StateSpaceConstants.DoubleJointedArm.armMaxRad) {
			return StateSpaceConstants.DoubleJointedArm.armMaxRad;
		}
		return armRad;
	}

	public double limitElbowRad(double elbowRad) {
		if (elbowRad < StateSpaceConstants.DoubleJointedArm.elbowMinRad) {
			return StateSpaceConstants.DoubleJointedArm.elbowMinRad;
		} else if (elbowRad > StateSpaceConstants.DoubleJointedArm.elbowMaxRad) {
			return StateSpaceConstants.DoubleJointedArm.elbowMaxRad;
		}
		return elbowRad;
	}
	public boolean isArmCharacterizationAtLimit(){
		if (doubleJointedArmInputs.positionArmRads > StateSpaceConstants.DoubleJointedArm.armMaxRad) {
			return true;
		} else if (doubleJointedArmInputs.positionArmRads < StateSpaceConstants.DoubleJointedArm.armMinRad) {
			return true;
		}
		return false;
	}
	public boolean isElbowCharacterizationAtLimit(){
		if (doubleJointedArmInputs.positionElbowRads > StateSpaceConstants.DoubleJointedArm.elbowMaxRad) {
			return true;
		} else if (doubleJointedArmInputs.positionElbowRads < StateSpaceConstants.DoubleJointedArm.elbowMinRad) {
			return true;
		}
		return false;
	}
	public void runArmVolts(double armVolts){
		List<Double> volts = new ArrayList<>();
		volts.add(armVolts);
		volts.add(0.0); // elbow
		currentState = State.CHARACTERIZATION;
		doubleJointedArmIO.setVoltage(volts);
	}
	public void runElbowVolts(double elbowVolts){
		List<Double> volts = new ArrayList<>();
		volts.add(0.0); // arm
		volts.add(elbowVolts);
		currentState = State.CHARACTERIZATION;
		doubleJointedArmIO.setVoltage(volts);
	}
	public void endCharacterization(){
		doubleJointedArmIO.stop();
		currentState = State.PERIODIC;
	}
	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("DoubleArmMotorTemp", doubleJointedArmInputs.armTemp);
		tempMap.put("DoubleElbowMotorTemp", doubleJointedArmInputs.elbowTemp);
		return tempMap;
	}

	@Override
	public void setCurrentLimit(int amps) {
		doubleJointedArmIO.setCurrentLimit(amps);
	}

	public double getArmRads() {
		return doubleJointedArmInputs.positionArmRads;
	}

	public double getElbowRads() {
		return doubleJointedArmInputs.positionElbowRads;
	}
	public double getArmRadsPerSec(){
		return doubleJointedArmInputs.velocityArmRadsPerSec;
	}
	public double getElbowRadsPerSec(){
		return doubleJointedArmInputs.velocityElbowRadsPerSec;
	}
	/**
	 * @return a double list which contains an x coordinate in meters for the
	 *         endpoint, and y.
	 */
	public double[] getCoordinate() {
		double armPos = getArmRads();
		double elbowPos = getElbowRads() + getArmRads();
		double x = Math.cos(armPos)
				* StateSpaceConstants.DoubleJointedArm.armLength
				+ Math.cos(elbowPos)
						* StateSpaceConstants.DoubleJointedArm.elbowLength;
		double y = Math.sin(armPos)
				* StateSpaceConstants.DoubleJointedArm.armLength
				+ Math.sin(elbowPos)
						* StateSpaceConstants.DoubleJointedArm.elbowLength;
		return new double[] { x, y
		};
	}

	@Override
	public double getCurrent() {
		return Math.abs(doubleJointedArmInputs.currentAmps[0]) + Math.abs(doubleJointedArmInputs.currentAmps[1]);
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(run(() -> DataHandler.logData(
				StateSpaceConstants.DoubleJointedArm.macroTopRight,
				"DoubleJointSetpoint")).withTimeout(5), runOnce(() -> {
					double[] coordinate = getCoordinate();
					if (Math.abs(coordinate[0]
							- StateSpaceConstants.DoubleJointedArm.macroTopRight[0]) > Units
									.inchesToMeters(5)
							|| Math.abs(coordinate[1]
									- StateSpaceConstants.DoubleJointedArm.macroTopRight[1]) > Units
											.inchesToMeters(5)) {
						addFault(
								"[System Check] Arm position was off more than 5 in. Wanted 1.5,1.1, got "
										+ coordinate[0] + "," + coordinate[1],
								false, true);
					}
				}),
				run(() -> DataHandler.logData(
						StateSpaceConstants.DoubleJointedArm.macroTopLeft,
						"DoubleJointSetpoint")).withTimeout(5),
				runOnce(() -> {
					double[] coordinate = getCoordinate();
					if (Math.abs(coordinate[0]
							- StateSpaceConstants.DoubleJointedArm.macroTopLeft[0]) > Units
									.inchesToMeters(5)
							|| Math.abs(coordinate[1]
									- StateSpaceConstants.DoubleJointedArm.macroTopLeft[1]) > Units
											.inchesToMeters(5)) {
						addFault(
								"[System Check] Arm position was off more than 5 in. Wanted -1.5,1.0, got "
										+ coordinate[0] + "," + coordinate[1],
								false, true);
					}
				})).until(() -> !getFaults().isEmpty());
	}
}
