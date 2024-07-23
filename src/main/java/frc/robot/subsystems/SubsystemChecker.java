package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.selfCheck.*;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingNavX2;
import frc.robot.utils.selfCheck.drive.SelfCheckingPWMMotor;
import frc.robot.utils.selfCheck.drive.SelfCheckingPigeon2;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public abstract class SubsystemChecker extends SubsystemBase {
	public enum SystemStatus {
		OK, WARNING, ERROR
	}

	private final List<SubsystemFault> faults = new ArrayList<>();
	private final List<SelfChecking> hardware = new ArrayList<>();
	private final String statusTable;
	private final boolean checkErrors;

	public SubsystemChecker() {
		this.statusTable = "SystemStatus/" + this.getName();
		Command systemCheck = getSystemCheckCommand();
		systemCheck.setName(getName() + "Check");
		SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
		Logger.recordOutput(statusTable + "/CheckRan", false);
		checkErrors = RobotBase.isReal();
		setupCallbacks();
	}

	public SubsystemChecker(String name) {
		this.setName(name);
		this.statusTable = "SystemStatus/" + name;
		Command systemCheck = getSystemCheckCommand();
		systemCheck.setName(getName() + "Check");
		SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
		Logger.recordOutput(statusTable + "/CheckRan", false);
		checkErrors = RobotBase.isReal();
		setupCallbacks();
	}

	public Command getSystemCheckCommand() {
		return Commands.sequence(Commands.runOnce(() -> {
			Logger.recordOutput(statusTable + "/CheckRan", false);
			clearFaults();
			publishStatus();
		}), systemCheckCommand(), Commands.runOnce(() -> {
			publishStatus();
			Logger.recordOutput(statusTable + "/CheckRan", true);
		}));
	}

	public abstract List<ParentDevice> getOrchestraDevices();

	public abstract double getCurrent();
	public abstract HashMap<String, Double> getTemps();
	public abstract void setCurrentLimit(int amps);
	private void setupCallbacks() {
		Robot.addPeriodic(this::checkForFaults, 0.25);
		Robot.addPeriodic(this::publishStatus, 1.0);
	}

	private void publishStatus() {
		SystemStatus status = getSystemStatus();
		Logger.recordOutput(statusTable + "/Status", status.name());
		Logger.recordOutput(statusTable + "/SystemOK", status == SystemStatus.OK);
		String[] faultStrings = new String[this.faults.size()];
		for (int i = 0; i < this.faults.size(); i++) {
			SubsystemFault fault = this.faults.get(i);
			faultStrings[i] = String.format("[%.2f] %s", fault.timestamp,
					fault.description);
		}
		Logger.recordOutput(statusTable + "/Faults", faultStrings);
		if (faultStrings.length > 0) {
			Logger.recordOutput(statusTable + "/LastFault",
					faultStrings[faultStrings.length - 1]);
		} else {
			Logger.recordOutput(statusTable + "/LastFault", "");
		}
	}

	protected void addFault(SubsystemFault fault) {
		faults.remove(fault);
		faults.add(fault);
	}

	protected void addFault(String description, boolean isWarning) {
		this.addFault(new SubsystemFault(description, isWarning));
	}

	protected void addFault(String description, boolean isWarning,
			boolean sticky) {
		this.addFault(new SubsystemFault(description, isWarning, sticky));
	}

	protected void addFault(String description) {
		this.addFault(description, false);
	}

	public List<SubsystemFault> getFaults() { return this.faults; }

	public void clearFaults() { this.faults.clear(); }

	public SystemStatus getSystemStatus() {
		SystemStatus worstStatus = SystemStatus.OK;
		for (SubsystemFault f : this.faults) {
			if (f.sticky || f.timestamp > Logger.getTimestamp() - 10) {
				if (f.isWarning) {
					if (worstStatus != SystemStatus.ERROR) {
						worstStatus = SystemStatus.WARNING;
					}
				} else {
					worstStatus = SystemStatus.ERROR;
				}
			}
		}
		return worstStatus;
	}

	public void registerHardware(String label, TalonFX talon) {
		hardware.add(new SelfCheckingTalonFX(label, talon));
	}

	public void registerHardware(String label, PWMMotorController pwmMotor) {
		hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
	}

	public void registerHardware(String label, CANSparkBase sparkBase) {
		hardware.add(new SelfCheckingSparkBase(label, sparkBase));
	}

	public void registerHardware(String label, AHRS navX) {
		hardware.add(new SelfCheckingNavX2(label, navX));
	}

	public void registerHardware(String label, Pigeon2 pigeon2) {
		hardware.add(new SelfCheckingPigeon2(label, pigeon2));
	}

	public void registerHardware(String label, CANcoder canCoder) {
		hardware.add(new SelfCheckingCANCoder(label, canCoder));
	}

	public void registerHardware(String label, LaserCan laserCan) {
		hardware.add(new SelfCheckingLaserCAN(label, laserCan));
	}

	public void registerAllHardware(List<SelfChecking> selfCheckingDevices) {
		hardware.addAll(selfCheckingDevices);
	}

	// Command to run a full systems check
	protected abstract Command systemCheckCommand();

	// Method to check for faults while the robot is operating normally
	private void checkForFaults() {
		if (checkErrors) {
			for (SelfChecking device : hardware) {
				for (SubsystemFault fault : device.checkForFaults()) {
					addFault(fault);
				}
			}
		}
	}
}
