package frc.robot.subsystems.drive.FastSwerve;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.TuningConstants;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.Logger;


public class Module {
	private static final LoggableTunedNumber drivekP = new LoggableTunedNumber(
			"Drive/Module/DrivekP",
			DriveConstants.overallDriveMotorConstantContainer
					.getP(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber drivekI = new LoggableTunedNumber(
			"Drive/Module/DrivekI",
			DriveConstants.overallDriveMotorConstantContainer
					.getI(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber drivekD = new LoggableTunedNumber(
			"Drive/Module/DrivekD",
			DriveConstants.overallDriveMotorConstantContainer
					.getD(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber drivekS = new LoggableTunedNumber(
			"Drive/Module/DrivekS",
			DriveConstants.overallDriveMotorConstantContainer
					.getKs(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber drivekV = new LoggableTunedNumber(
			"Drive/Module/DrivekV",
			DriveConstants.overallDriveMotorConstantContainer
					.getKv(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnkP = new LoggableTunedNumber(
			"Drive/Module/TurnkP",
			DriveConstants.overallTurningMotorConstantContainer
					.getP(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnkI = new LoggableTunedNumber(
			"Drive/Module/TurnkI",
			DriveConstants.overallTurningMotorConstantContainer
					.getI(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnkD = new LoggableTunedNumber(
			"Drive/Module/TurnkD",
			DriveConstants.overallTurningMotorConstantContainer
					.getD(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnkS = new LoggableTunedNumber(
			"Drive/Module/TurnkS",
			DriveConstants.overallTurningMotorConstantContainer
					.getKs(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnkV = new LoggableTunedNumber(
			"Drive/Module/TurnkV",
			DriveConstants.overallTurningMotorConstantContainer
					.getKv(), TuningConstants.isTuningDrivetrain);
	private static final LoggableTunedNumber turnDeadband = new LoggableTunedNumber("Drive/Module/TurnDeadband",
			DriveConstants.TURN_DEADBAND_AMPS, TuningConstants.isTuningDrivetrain);
	private SwerveModuleState setpointState = new SwerveModuleState();
	private final int index;
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
			DriveConstants.overallDriveMotorConstantContainer
					.getKs(),
			DriveConstants.overallDriveMotorConstantContainer
					.getKv(),
			0.0);
	public final String name;

	public Module(ModuleIO io, int index) {
		this.io = io;
		this.index = index;
		switch (index) {
			case 0:
				name = "FrontLeftModule";
				break;
			case 1:
				name = "FrontRightModule";
				break;
			case 2:
				name = "BackLeftModule";
				break;
			case 3:
				name = "BackRightModule";
				break;
			default:
				name = "Unknown";
				break;
		}
	}

	/** Called while blocking odometry thread */
	public void updateInputs() {
		io.updateInputs(inputs);
		/*inputs.turnAbsolutePosition = inputs.turnAbsolutePosition
				.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		inputs.turnPosition = inputs.turnPosition.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		for (Rotation2d value : inputs.odometryTurnPositions) {
			value.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		}*/
		Logger.processInputs("Drive/Module" + index, inputs);
		// Update ff and controllers
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> ff = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(),
						0),
				drivekS, drivekV);
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> io.setDrivePID(drivekP.get(), drivekI.get(), drivekD.get(), drivekS.get(), drivekV.get()),
				drivekP, drivekI, drivekD, drivekS, drivekV);
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> io.setTurnPID(turnkP.get(), turnkI.get(), turnkD.get(), turnkS.get(), turnkV.get(), turnDeadband.get()),
				turnkP, turnkI, turnkD, turnkS, turnkV,turnDeadband);
	}

	public void shift(boolean lowGear) {
		io.shift(lowGear);
	}

	/** Runs to {@link SwerveModuleState} */
	public void runSetpoint(SwerveModuleState setpoint, SwerveModuleState wheelNM) {
    setpointState = setpoint;
    Logger.recordOutput("Drive/SwerveSetpoint", setpointState.speedMetersPerSecond);

    double setpointWheelAngularVel = setpoint.speedMetersPerSecond / (DriveConstants.TrainConstants.kWheelDiameter.get() / 2.0);
    double currentWheelAngularVel = getVelocityMetersPerSec() / (DriveConstants.TrainConstants.kWheelDiameter.get() / 2.0);
	double motorTorqueNm = wheelNM.speedMetersPerSecond;
    // Now use vendor-specific conversions as before:
    if ((DriveConstants.robotMotorController == MotorVendor.CTRE_ON_CANIVORE
            || DriveConstants.robotMotorController == MotorVendor.CTRE_ON_RIO)
            && Constants.currentMode != Mode.SIM) {

        if (Constants.currentMatchState == FRCMatchState.AUTO
                || Constants.currentMatchState == FRCMatchState.AUTOINIT) {
            io.runDriveVelocitySetpoint(
                    setpointWheelAngularVel,
                    ff.calculate(currentWheelAngularVel)
            + ((motorTorqueNm / DriveConstants.TrainConstants.kDriveMotorGearRatioHigh) * DriveConstants.getDriveTrainMotors(1,DriveConstants.TrainConstants.kDriveMotorGearRatioHigh).KtNMPerAmp));
        } else {
            io.runDriveVelocitySetpoint(
                    setpointWheelAngularVel,
                    (inputs.negateFF ? 0 : 1) * ((motorTorqueNm / DriveConstants.TrainConstants.kDriveMotorGearRatioHigh) * DriveConstants.getDriveTrainMotors(1,DriveConstants.TrainConstants.kDriveMotorGearRatioHigh).KtNMPerAmp)
                            + ff.calculate(currentWheelAngularVel));
        }

    } else {
        double wheelTorqueVolts = DriveConstants.getDriveTrainMotors(1).getVoltage(motorTorqueNm, setpointWheelAngularVel);
        Logger.recordOutput("Drive/" + name + "/wheelTorqueVolts", wheelTorqueVolts);
        io.runDriveVelocitySetpoint(
                setpointWheelAngularVel,
                (inputs.negateFF ? 0 : 1) * ff.calculateWithVelocities(currentWheelAngularVel, setpointWheelAngularVel)
                        + (wheelTorqueVolts));
    }

    io.runTurnPositionSetpoint(setpoint.angle.getRadians());
}


	/**
	 * Runs characterization volts or amps depending on using voltage or current
	 * control.
	 */
	public void runCharacterization(double turnSetpointRads, double input) {
		io.runTurnPositionSetpoint(turnSetpointRads);
		io.runDriveVolts(input);
	}

	/** Sets brake mode to {@code enabled}. */
	public void setBrakeMode(boolean enabled) {
		if (inputs.driveMotorConnected) {
			io.setDriveBrakeMode(enabled);
		} else {
			io.setDriveBrakeMode(false);
		}
		if (inputs.turnMotorConnected) {
			io.setTurnBrakeMode(enabled);
		} else {
			io.setTurnBrakeMode(false);
		}
	}

	public void setCurrentLimit(int amps) {
		io.setCurrentLimit(amps);
	}

	/** Stops motors. */
	public void stop() {
		io.stop();
	}

	/** Get all latest {@link SwerveModulePosition}'s from last cycle. */
	public SwerveModulePosition[] getModulePositions() {
		int minOdometryPositions = Math.min(
				inputs.odometryDrivePositionsMeters.length,
				inputs.odometryTurnPositions.length);
		SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
		for (int i = 0; i < minOdometryPositions; i++) {
			positions[i] = new SwerveModulePosition(
					inputs.odometryDrivePositionsMeters[i],
					inputs.odometryTurnPositions[i]);
		}
		return positions;
	}

	/** Get the current shift state as boolean */
	public boolean inLowGear() {
		return inputs.inLowGear;
	}

	/** Get the current RPS of the motor ROTOR */
	public double getDriveMotorRPM() {
		return inputs.driveRotorRPM;
	}

	/** Get turn angle of module as {@link Rotation2d}. */
	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	/** Get position of wheel rotations in radians */
	public double getPositionRads() {
		return inputs.drivePositionRads;
	}

	/** Get position of wheel in meters. */
	public double getPositionMeters() {
		return inputs.drivePositionRads
				* (DriveConstants.TrainConstants.kWheelDiameter.get() / 2);
	}

	/** Get velocity of wheel in m/s. */
	public double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadsPerSec
				* (DriveConstants.TrainConstants.kWheelDiameter.get() / 2);
	}

	/** Get current {@link SwerveModulePosition} of module. */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Get current {@link SwerveModuleState} of module. */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Get velocity of drive wheel for characterization */
	public double getCharacterizationVelocity() {
		return inputs.driveVelocityRadsPerSec;
	}

	/** Returns the temp of the DRIVE motor in C */
	public double getDriveMotorTemp() {
		return inputs.driveMotorTemp;
	}

	public double getCurrent() {
		return inputs.driveSupplyCurrentAmps + inputs.turnSupplyCurrentAmps;
	}

	/** Returns the temp of the DRIVE motor in C */
	public double getTurnMotorTemp() {
		return inputs.turnMotorTemp;
	}

	public SwerveModuleState getSetpointState() {
		return setpointState;
	}

	public boolean isDriveConnected() {
		return inputs.driveMotorConnected;
	}

	public boolean isTurnConnected() {
		return inputs.turnMotorConnected;
	}

	public List<SelfChecking> getSelfCheckingHardware() {
		return io.getSelfCheckingHardware();
	}
	public void changeDeadband(double amps){
		io.changeDeadband(amps);
	}
}
