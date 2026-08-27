package frc.robot.commands.drive;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.Swerve.ModuleLimits;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.GeomUtil;
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
			"Drive/Translational Response Curve Exponent",
			DriveConstants.DriverConstants.translationalResponseCurveExponent, TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber rotationalResponseCurve = new LoggableTunedNumber(
			"Drive/Rotational Response Curve Exponent", DriveConstants.DriverConstants.rotationalResponseCurveExponent,
			TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber deadzone = new LoggableTunedNumber("Drive/Deadzone",
			DriveConstants.DriverConstants.kDeadband, TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber rotationalSpeedMaxPercentage = new LoggableTunedNumber(
			"Drive/RotationalSpeedMaxPercentage", .75, TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber autoIntakeAssistPercentage = new LoggableTunedNumber(
			"Drive/AutoIntakeAssistPercentage", .5, TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber autoIntakeWallSlowDistanceMeters = new LoggableTunedNumber(
			"Drive/AutoIntakeWallSlowDistanceMeters", 0.9, TuningConstants.isTuningDrivetrain);
	static final LoggableTunedNumber autoIntakeWallMaxApproachSpeedMetersPerSec = new LoggableTunedNumber(
			"Drive/AutoIntakeWallMaxApproachSpeedMetersPerSec", 1.2, TuningConstants.isTuningDrivetrain);
	private static final double TELEOP_SHOOT_ACCEL_SCALE = 1.0 / 6.0;
	private static final double SHOOT_TRIGGER_FULL_THRESHOLD = 0.875;
	private Function<Double, Double> translationalCurve = ResponseCurve.QUADRATIC;
	private Function<Double, Double> rotationalCurve = ResponseCurve.SOFT;

	private Command activeAimCommand = null;
	private boolean aimInitialized = false;

	public DrivetrainC(DrivetrainS drivetrainS) {
		this.drivetrainS = drivetrainS;
		controller = new TunedJoystick(RobotContainer.driveController.getHID());
		controller.setDeadzone(deadzone.get());
		addRequirements(drivetrainS);

	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		updateTeleopShootAccelerationLimit();
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


		if (DriveConstants.autoIntake && activeAimCommand != null && aimInitialized) {
			try {
				activeAimCommand.execute();
			} catch (Exception e) {
				System.out.println("DrivetrainC error executing AimToRotation: " + e.toString());
				try {
					if (activeAimCommand != null)
						activeAimCommand.end(true);
				} catch (Exception ex) {
				}
				activeAimCommand = null;
				aimInitialized = false;
			}
		}
		if (RobotContainer.userDrive) {
			if (DriveConstants.driveType == DriveConstants.DriveTrainType.TANK) {
				turningSpeed = turningSpeed
						* DriveConstants.kMaxSpeedMetersPerSecond;
			} else {
				turningSpeed = turningSpeed
						* DriveConstants.kMaxTurningSpeedRadPerSec * rotationalSpeedMaxPercentage.get();
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
				if (RobotContainer.withinLineTolerance) {
					if (RobotContainer.drivetrainS.getLookAheadPose().getY() >= FieldConstants.FIELD_HEIGHT / 2) {
						if (Robot.isRed) {
							double xVal = ySpeed * Math.cos(Math.PI / 2);
							double yVal = ySpeed * Math.sin(Math.PI / 2);
							xVal += xSpeed;
							chassisSpeeds = new ChassisSpeeds(-xVal + RobotContainer.xSpeed,
									-yVal + RobotContainer.ySpeed, 0);
						} else {
							double xVal = ySpeed * Math.cos(Math.PI / 2);
							double yVal = ySpeed * Math.sin(Math.PI / 2);
							xVal += xSpeed;
							chassisSpeeds = new ChassisSpeeds(xVal + RobotContainer.xSpeed,
									yVal + RobotContainer.ySpeed, 0);
						}

					} else {
						if (Robot.isRed) {
							double xVal = ySpeed * Math.cos(Math.PI / 2);
							double yVal = ySpeed * Math.sin(Math.PI / 2);
							xVal += xSpeed;
							chassisSpeeds = new ChassisSpeeds(-xVal + RobotContainer.xSpeed,
									-yVal + RobotContainer.ySpeed, 0);
						} else {
							double xVal = ySpeed * Math.cos(Math.PI / 2);
							double yVal = ySpeed * Math.sin(Math.PI / 2);
							xVal += xSpeed;
							chassisSpeeds = new ChassisSpeeds(xVal + RobotContainer.xSpeed,
									yVal + RobotContainer.ySpeed, 0);
						}
					}
				} else {
					if (RobotContainer.xSpeed != 0) {
						xSpeed = RobotContainer.xSpeed + xSpeed;
					}
					if (RobotContainer.ySpeed != 0) {
						ySpeed = RobotContainer.ySpeed + ySpeed;
					}
					chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
							turningSpeed, drivetrainS.getRotation2d());
				}
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
				Logger.recordOutput("Controler/SetY", ySpeed);
				drivetrainS.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));// for odom
				drivetrainS.stopModules();
				} else {
					// Deal with opposing robots.
					if (DriveConstants.autoAvoidance) {
						chassisSpeeds = GeomUtil.avoidRobots(chassisSpeeds);
					}
					// Deal with FUEL
					/*if (DriveConstants.autoIntake) {
						Pose2d ourPose = drivetrainS.getLookAheadPose();
						if (ourPose != null && preferredFuelObservation.isPresent()) {
							ObjDetectTxyObservation fuelObservation = preferredFuelObservation.get().observation();
							Translation2d fuelTarget = GeomUtil.projectObjectObservationToField(
									ourPose,
									VisionConstants.cameras[CameraID.INTAKE_CAM.ordinal()].getPose().get(),
									fuelObservation.tx(),
									fuelObservation.ty(),
									fuelObservation.distanceMeters());
							Translation2d robotToFuel = fuelTarget.minus(ourPose.getTranslation());
							double dist = robotToFuel.getNorm();
							if (dist > 1e-6) {
								Rotation2d robotRot = drivetrainS.getRotation2d();
								Translation2d robotRelativeFuel = robotToFuel.rotateBy(robotRot.unaryMinus());
								double rx = robotRelativeFuel.getX() / dist;

								double driverVx = chassisSpeeds.vxMetersPerSecond;
								double driverMag = Math.hypot(
										chassisSpeeds.vxMetersPerSecond,
										chassisSpeeds.vyMetersPerSecond);
								double desiredVx = Math.signum(rx) * driverMag;

								double k = autoIntakeAssistPercentage.get();
								double correctedVx = MathUtil.interpolate(driverVx, desiredVx, k);

								ChassisSpeeds assistedSpeeds = new ChassisSpeeds(
										correctedVx,
										chassisSpeeds.vyMetersPerSecond,
										chassisSpeeds.omegaRadiansPerSecond);
								Translation3d fieldRelativeVelocity3d = new Translation3d(
										assistedSpeeds.vxMetersPerSecond,
										assistedSpeeds.vyMetersPerSecond,
										0.0).rotateBy(new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, robotRot.getRadians()));
								Translation2d limitedFieldVelocity = GeomUtil.limitVelocityTowardFieldEdge(
										fieldRelativeVelocity3d.toTranslation2d(),
										ourPose.getTranslation(),
										fuelTarget,
										autoIntakeWallSlowDistanceMeters.get(),
										autoIntakeWallMaxApproachSpeedMetersPerSec.get());
								Translation2d limitedRobotVelocity = limitedFieldVelocity.rotateBy(robotRot.unaryMinus());

								chassisSpeeds = new ChassisSpeeds(
										limitedRobotVelocity.getX(),
										limitedRobotVelocity.getY(),
										assistedSpeeds.omegaRadiansPerSecond);
							}
							Logger.recordOutput("Drive/AutoIntakeAssist/FuelTarget",
									new Pose2d(fuelTarget, ourPose.getRotation()));
							Logger.recordOutput("Drive/AutoIntakeAssist/ClusterCount",
									preferredFuelObservation.get().clusterCount());
							Logger.recordOutput("Drive/AutoIntakeAssist/ClusterScore",
									preferredFuelObservation.get().clusterScore());
							Logger.recordOutput("Drive/AutoIntakeAssist/FuelTxDeg", fuelObservation.tx().getDegrees());
							Logger.recordOutput("Drive/AutoIntakeAssist/FuelTyDeg", fuelObservation.ty().getDegrees());
							Logger.recordOutput("Drive/AutoIntakeAssist/FuelDistanceMeters",
									fuelObservation.distanceMeters());
						}*/
					}
				Logger.recordOutput("Controller/autoAvoidance", DriveConstants.autoAvoidance);
				Logger.recordOutput("Controller/SetTurn", turningSpeed);
				if (RobotContainer.withinLineTolerance) {
					System.out.println("Within Line Tolerance");
				}
				Logger.recordOutput("Controller/SetX", xSpeed);
				Logger.recordOutput("Controller/SetY", ySpeed);
				drivetrainS.setChassisSpeeds(chassisSpeeds);
			}
		}

	private void updateTeleopShootAccelerationLimit() {
		if (!(drivetrainS instanceof Swerve swerve)) {
			return;
		}

		boolean shouldLimitAccel = DriverStation.isTeleopEnabled()
				&& RobotContainer.driveController.getHID().getRightTriggerAxis() >= SHOOT_TRIGGER_FULL_THRESHOLD;
		ModuleLimits desiredLimits = shouldLimitAccel
				? new ModuleLimits(
						DriveConstants.moduleLimitsLow.maxDriveVelocity(),
						DriveConstants.moduleLimitsLow.maxDriveAcceleration() * TELEOP_SHOOT_ACCEL_SCALE,
						DriveConstants.moduleLimitsLow.maxSteeringVelocity())
				: DriveConstants.moduleLimitsLow;

		Logger.recordOutput("Drive/TeleopShootAccelLimited", shouldLimitAccel);
		Logger.recordOutput("Drive/TeleopShootAccelScale", shouldLimitAccel ? TELEOP_SHOOT_ACCEL_SCALE : 1.0);

		if (!desiredLimits.equals(swerve.getModuleLimits())) {
			swerve.setCurrentModuleLimits(desiredLimits);
		}
	}

	@Override
	public void end(boolean interrupted) {
		// make sure to end any active aim if present
		if (activeAimCommand != null && aimInitialized) {
			try {
				activeAimCommand.end(interrupted);
			} catch (Exception e) {
			}
		}
		activeAimCommand = null;
		aimInitialized = false;
		if (drivetrainS instanceof Swerve swerve && !DriveConstants.moduleLimitsLow.equals(swerve.getModuleLimits())) {
			swerve.setCurrentModuleLimits(DriveConstants.moduleLimitsLow);
		}

		drivetrainS.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
