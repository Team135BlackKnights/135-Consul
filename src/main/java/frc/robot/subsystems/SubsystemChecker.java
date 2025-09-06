package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.selfCheck.*;
import frc.robot.utils.selfCheck.drive.SelfCheckingCANCoder;
import frc.robot.utils.selfCheck.drive.SelfCheckingNavX2;
import frc.robot.utils.selfCheck.drive.SelfCheckingPWMMotor;
import frc.robot.utils.selfCheck.drive.SelfCheckingPigeon2;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.littletonrobotics.junction.Logger;

public abstract class SubsystemChecker extends SubsystemBase {
	public enum SystemStatus {
		OK, WARNING, ERROR
	}

	private final ConcurrentLinkedQueue<SubsystemFault> faults = new ConcurrentLinkedQueue<>();
	private final ConcurrentLinkedQueue<SelfChecking> hardware = new ConcurrentLinkedQueue<>();
	private final String statusTable;
	private boolean checkErrors;

	public SubsystemChecker() {
		System.out.println(this.getName());
		this.statusTable = "SystemStatus/" + this.getName();
		checkErrors = true;
		setupCallbacks();
	}

	public SubsystemChecker(String name) {
		this.setName(name);
		this.statusTable = "SystemStatus/" + name;
		checkErrors = true;
		setupCallbacks();
	}
	public void setupSystemCheck(){
		Command systemCheck = getSystemCheckCommand();
		systemCheck.setName(getName() + "Check");
		SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
		Logger.recordOutput(statusTable + "/CheckRan", false);
	}
	/**
	 * Do not call before all systems are set up, as they make sure they aren't in bad states.
	 * @return
	 */
	public Command getSystemCheckCommand() {
		return Commands.sequence(Commands.runOnce(() -> {
			Logger.recordOutput(statusTable + "/CheckRan", false);
			clearFaults();
			publishStatus(true);
		}), systemCheckCommand(), Commands.runOnce(() -> {
			publishStatus(true);
			Logger.recordOutput(statusTable + "/CheckRan", true);
		}));
	}

	public abstract List<ParentDevice> getOrchestraDevices();

	public abstract double getCurrent();

	public abstract HashMap<String, Double> getTemps();

	public abstract void setCurrentLimit(int amps);

	private void setupCallbacks() {
		Robot.addPeriodic(() -> checkForFaults(false), 0.5);
		Robot.addPeriodic(() -> publishStatus(false), 1.5);
	}

	//Elastic.Notification currentNotif;
	Timer lastSentFaultTimer = new Timer();

	private void publishStatus(boolean override) {
		if (checkErrors || override) {
			SystemStatus status = getSystemStatus();
			Logger.recordOutput(statusTable + "/Status", status.name());
			Logger.recordOutput(statusTable + "/SystemOK",
					status == SystemStatus.OK);
			String[] faultStrings = new String[this.faults.size()];
			int i = 0;
			for (SubsystemFault fault : this.faults) {
				faultStrings[i] = String.format("[%.2f] %s", fault.timestamp,
						fault.description);
				i++; // doing seperate from for loop to avoid concurrent modification exception
			}
			Logger.recordOutput(statusTable + "/Faults", faultStrings);
			if (faultStrings.length > 0) {
				Logger.recordOutput(statusTable + "/LastFault",
						faultStrings[faultStrings.length - 1]);
				/*if (currentNotif == null) {
					currentNotif = new Elastic.Notification(
							this.faults.element().isWarning ? NotificationLevel.WARNING : NotificationLevel.ERROR,
							statusTable + ": " + status.name(), faultStrings[faultStrings.length - 1],
							this.faults.element().sticky ? 10000 : 3000).withAutomaticHeight();
					Elastic.sendNotification(currentNotif);
					lastSentFaultTimer.restart();
				} else {
					// get after the first ] to the end of the string
					String faultString = faultStrings[faultStrings.length - 1].split("]")[1];
					if (!faultString.equals(currentNotif.getDescription().split("]")[1])
							|| lastSentFaultTimer.hasElapsed(currentNotif.getDisplayTimeMillis() / 1000 + 1)) { // new
																												// fault
						currentNotif = new Elastic.Notification(
								this.faults.element().isWarning ? NotificationLevel.WARNING : NotificationLevel.ERROR,
								statusTable + ": " + status.name(), faultStrings[faultStrings.length - 1],
								this.faults.element().sticky ? 10000 : 3000).withAutomaticHeight();
						Elastic.sendNotification(currentNotif);
						lastSentFaultTimer.restart();
					}

					// currentNotif.setMessage(String.join("\n", faultStrings));
				}*/
			} else {
				Logger.recordOutput(statusTable + "/LastFault", "");
			}
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

	public void allowFaultPolling(boolean checkFaults) {
		this.checkErrors = checkFaults;
	}

	public ConcurrentLinkedQueue<SubsystemFault> getFaults() {
		return this.faults;
	}

	public void clearFaults() {
		this.faults.clear();
	}

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

	public void registerHardware(String label, SparkBase sparkBase) {
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
	private void checkForFaults(boolean override) {
		if (checkErrors || override) {
			hardware.forEach(
					device -> device.checkForFaults().forEach(this::addFault));
		}
	}
}
