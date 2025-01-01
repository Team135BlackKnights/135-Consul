package frc.robot.commands.drive;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.TunedJoystick;
import frc.robot.utils.drive.TunedJoystick.ResponseCurve;

/*
 * Use this as explanation on how to pull DataLogs:
 * https://docs.wpilib.org/en/stable/ docs/software/telemetry/datalog-
 * download.html Should theoretically output a csv file, see if we can convert
 * it into a txt file and upload it to smth
 */
public class DrivetrainC extends Command {
	public ChassisSpeeds chassisSpeeds;
	private final DrivetrainS drivetrainS;
	private final TunedJoystick controller;
	static final LoggableTunedNumber translationalResponseCurve = new LoggableTunedNumber(
			"Drive/Translational Response Curve Exponent", DriveConstants.DriverConstants.translationalResponseCurveExponent);
	static final LoggableTunedNumber rotationalResponseCurve = new LoggableTunedNumber(
			"Drive/Rotational Response Curve Exponent", DriveConstants.DriverConstants.rotationalResponseCurveExponent);
	static final LoggableTunedNumber deadzone = new LoggableTunedNumber("Drive/Deadzone", DriveConstants.DriverConstants.kDeadband);
	private Function<Double, Double> translationalCurve = ResponseCurve.QUADRATIC;
	private Function<Double, Double> rotationalCurve = ResponseCurve.SOFT;

	public DrivetrainC(DrivetrainS drivetrainS) {
		this.drivetrainS = drivetrainS;
		controller = new TunedJoystick(RobotContainer.driveController);
		controller.setDeadzone(deadzone.get());
		addRequirements(drivetrainS);

	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		// update the curves
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			translationalCurve = val -> Math.pow(val, translationalResponseCurve.get());
			rotationalCurve = val -> Math.pow(val, rotationalResponseCurve.get());
		}, translationalResponseCurve, rotationalResponseCurve);
		// Update the deadzone
		LoggableTunedNumber.ifChanged(hashCode(), () -> {
			controller.setDeadzone(deadzone.get());
		}, deadzone);
		// Get the x, y, and rotational speeds from the joystick
		double xSpeed = -controller.getLeftY(translationalCurve);
		double ySpeed = -controller.getLeftX(translationalCurve);
		double turningSpeed = -controller.getRightX(rotationalCurve);
		xSpeed = xSpeed
				* DriveConstants.kMaxSpeedMetersPerSecond;
		ySpeed = ySpeed
				* DriveConstants.kMaxSpeedMetersPerSecond;
		if (DriveConstants.driveType == DriveConstants.DriveTrainType.TANK) {
			turningSpeed = turningSpeed
					* DriveConstants.kMaxSpeedMetersPerSecond;
		} else {
			turningSpeed = turningSpeed
					* DriveConstants.kMaxTurningSpeedRadPerSec;
		}
		if (Robot.isRed) {
			xSpeed *= -1;
			ySpeed *= -1;
			turningSpeed *= 1;
		}
		if (RobotContainer.angularSpeed != 0) {
			turningSpeed = RobotContainer.angularSpeed;
		}
		// Convert ChassisSpeeds into the ChassisSpeeds type
		if (DriveConstants.fieldOriented) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
					turningSpeed, drivetrainS.getRotation2d());
		} else {
			chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
		}
		if (DriveConstants.driveType == DriveConstants.DriveTrainType.TANK) {
			chassisSpeeds.vyMetersPerSecond = 0;
		}
		// set modules to proper speeds
		if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
			Logger.recordOutput("Controller/SetTurn", turningSpeed);
			Logger.recordOutput("Controller/SetX", xSpeed);
			Logger.recordOutput("Controller/SetY", ySpeed);
			drivetrainS.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));// for odom
			drivetrainS.stopModules();
		} else {
			Logger.recordOutput("Controller/SetTurn", turningSpeed);
			Logger.recordOutput("Controller/SetX", xSpeed);
			Logger.recordOutput("Controller/SetY", ySpeed);
			drivetrainS.setChassisSpeeds(chassisSpeeds);
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrainS.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
