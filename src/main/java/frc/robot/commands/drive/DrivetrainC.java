package frc.robot.commands.drive;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TuningConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.Swerve.TxTyPoseRecord;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.TunedJoystick;
import frc.robot.utils.drive.TunedJoystick.ResponseCurve;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.GeomUtil.ApproachDirection;

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
	private Function<Double, Double> translationalCurve = ResponseCurve.QUADRATIC;
	private Function<Double, Double> rotationalCurve = ResponseCurve.SOFT;

	private Command activeAimCommand = null; 
	private boolean aimInitialized = false; 
	private boolean lastAutoIntake = false; 

	public DrivetrainC(DrivetrainS drivetrainS) {
		this.drivetrainS = drivetrainS;
		controller = new TunedJoystick(RobotContainer.driveController);
		controller.setDeadzone(deadzone.get());
		addRequirements(drivetrainS);

	}

	private static final double MAX_CORAL_AGE_SECONDS = 3.0;

	private boolean isRecentValidCoral(TxTyPoseRecord coralRec) {
		if (coralRec == null)
			return false;
		Pose3d p3 = coralRec.pose();
		if (p3 == null)
			return false;

		double now = Timer.getFPGATimestamp();
		double age = now - coralRec.timestamp();
		if (age > MAX_CORAL_AGE_SECONDS)
			return false;

		// treat 0,0,0 as invalid since southmoon sometimes outputs that (idk why)
		if (p3.getTranslation().equals(Translation3d.kZero)) {
			return false;
		}

		return true;
	}

	private ChassisSpeeds avoidRobots(ChassisSpeeds speeds) {
		final double MAX_AGE_SECONDS = 2.0;
		final double BASE_AVOID_MARGIN_M = 0.5; // previous constant
		final double MAX_EXTRA_MARGIN_M = 1.2; // additional margin at top approach (tunable)
		final double MAX_AVOID_SPEED = DriveConstants.kMaxSpeedMetersPerSecond * 10;

		Pose2d ourPose = drivetrainS.getLookAheadPose();
		double now = Timer.getFPGATimestamp();

		double avoidRobotX = 0.0;
		double avoidRobotY = 0.0;
		boolean anyActive = false;

		double len = DriveConstants.kBumperToBumperLength;
		double wid = DriveConstants.kBumperToBumperWidth;

		double thetaLoop = drivetrainS.getRotation2d().getRadians();
		double cosLoop = Math.cos(-thetaLoop);
		double sinLoop = Math.sin(-thetaLoop);

		double maxDecel = DriveConstants.maxTranslationalAcceleration.get();

		// measured & commanded translational speed magnitude (global)
		ChassisSpeeds measured = drivetrainS.getChassisSpeeds();
		double measuredSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
		double commandedSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
		double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;

		// Log the globals at least
		Logger.recordOutput("Avoidance/MeasuredSpeed", measuredSpeed);
		Logger.recordOutput("Avoidance/CommandedSpeed", commandedSpeed);

		for (TxTyPoseRecord otherRobotPose : ((Swerve) drivetrainS).getOpposingRobotPoses()) {
			Pose3d other3 = otherRobotPose.pose();
			if (other3 == null)
				continue;
			double z = other3.getTranslation().getZ();
			if (z > VisionConstants.maxObjZError)
				continue;
			double age = now - otherRobotPose.timestamp();
			if (age > MAX_AGE_SECONDS)
				continue;

			double otherX = other3.getTranslation().getX();
			double otherY = other3.getTranslation().getY();

			double dx = ourPose.getX() - otherX;
			double dy = ourPose.getY() - otherY;
			double dist = Math.hypot(dx, dy);

			if (dist <= 1e-6) {
				dx = 1.0;
				dy = 0.0;
				dist = 1.0;
			}

			// field -> robot rotation unit vector for this target (unit from us->them)
			double uxField = (otherX - ourPose.getX()) / dist;
			double uyField = (otherY - ourPose.getY()) / dist;
			double uxRobot = cosLoop * uxField - sinLoop * uyField;
			double uyRobot = sinLoop * uxField + cosLoop * uyField;

			// compute approach-projection for commanded and measured velocities
			double cmdAlong = speeds.vxMetersPerSecond * uxRobot + speeds.vyMetersPerSecond * uyRobot;
			double measAlong = measured.vxMetersPerSecond * uxRobot + measured.vyMetersPerSecond * uyRobot;

			// Use the larger positive projection (if any) to decide "aiming"
			double approachAlong = Math.max(0.0, Math.max(cmdAlong, measAlong));

			// compute per-target dynamic margin: only expand if approachAlong > 0
			double margin;
			if (approachAlong <= 0.0) {
				margin = BASE_AVOID_MARGIN_M;
			} else {
				// scale extra margin by how big the approach is relative to max speed
				double approachScale = Math.min(1.0, approachAlong / Math.max(1e-6, maxSpeed));
				margin = BASE_AVOID_MARGIN_M + MAX_EXTRA_MARGIN_M * approachScale;
			}
			// clamp margin for safety
			margin = Math.max(BASE_AVOID_MARGIN_M, Math.min(BASE_AVOID_MARGIN_M + MAX_EXTRA_MARGIN_M, margin));

			Logger.recordOutput("Avoidance/PerTargetMargin", margin);
			Logger.recordOutput("Avoidance/PerTargetCmdAlong", cmdAlong);
			Logger.recordOutput("Avoidance/PerTargetMeasAlong", measAlong);

			double halfLen = (len * 0.5) + margin;
			double halfWid = (wid * 0.5) + margin;

			double absDx = Math.abs(dx);
			double absDy = Math.abs(dy);

			if (absDx < halfLen && absDy < halfWid) {
				double overlapX = Math.max(0.0, halfLen - absDx);
				double overlapY = Math.max(0.0, halfWid - absDy);

				double strengthX = Math.min(1.0, overlapX / halfLen);
				double strengthY = Math.min(1.0, overlapY / halfWid);

				double strength = Math.max(strengthX, strengthY);
				double mag = strength * MAX_AVOID_SPEED;

				Logger.recordOutput("Avoidance/OtherOverlapX", overlapX);
				Logger.recordOutput("Avoidance/OtherOverlapY", overlapY);
				Logger.recordOutput("Avoidance/OtherStrength", strength);

				// inward motion to consider (use max of cmd/meas)
				double inwardAlong = Math.max(0.0, Math.max(cmdAlong, measAlong));
				if (inwardAlong > 0.0) {
					double minOverlap = Math.min(overlapX, overlapY);

					// estimate stopping distance from current measured speed along approach
					double stoppingDist = (measAlong * measAlong) / (2.0 * Math.max(1e-3, maxDecel));
					Logger.recordOutput("Avoidance/StoppingDist", stoppingDist);
					Logger.recordOutput("Avoidance/MinOverlap", minOverlap);

					double desiredAlong;
					if (stoppingDist > minOverlap) {
						// emergency braking (unchanged behavior)
						double brakeVel = Math.min(maxSpeed, Math.max(0.5 * maxSpeed, measAlong));
						desiredAlong = -brakeVel;
						Logger.recordOutput("Avoidance/EmergencyBrake", brakeVel);
					} else {
						double cancel = Math.min(mag, inwardAlong);
						desiredAlong = cmdAlong - cancel;
					}

					double reduction = Math.max(0.0, cmdAlong - desiredAlong);
					reduction = Math.min(reduction, mag);

					avoidRobotX += -uxRobot * reduction;
					avoidRobotY += -uyRobot * reduction;
					anyActive = true;
					Logger.recordOutput("Avoidance/OtherAppliedReduction", reduction);
				}
				Logger.recordOutput("Avoidance/OtherAge", age);
			}
		} // end loop

		if (!anyActive) {
			return speeds;
		}

		double newVx = speeds.vxMetersPerSecond + avoidRobotX;
		double newVy = speeds.vyMetersPerSecond + avoidRobotY;

		double maxSpeedClamp = DriveConstants.kMaxSpeedMetersPerSecond;
		if (Math.abs(newVx) > maxSpeedClamp)
			newVx = Math.signum(newVx) * maxSpeedClamp;
		if (Math.abs(newVy) > maxSpeedClamp)
			newVy = Math.signum(newVy) * maxSpeedClamp;

		double newOmega = speeds.omegaRadiansPerSecond;

		ChassisSpeeds out = new ChassisSpeeds(newVx, newVy, newOmega);
		Logger.recordOutput("Avoidance/AppliedVX", avoidRobotX);
		Logger.recordOutput("Avoidance/AppliedVY", avoidRobotY);
		Logger.recordOutput("Avoidance/ResultVX", newVx);
		Logger.recordOutput("Avoidance/ResultVY", newVy);

		return out;
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

		if (DriveConstants.autoIntake && !lastAutoIntake) {
			// went false->true: try to create aim if coral exists
			if (drivetrainS instanceof Swerve) {
					Supplier<Pose2d> coralPoseSupplier = () ->{
						TxTyPoseRecord coralPose = ((Swerve) drivetrainS).getClosestCoralPose();
						if (isRecentValidCoral(coralPose)) {
							Pose2d pose = coralPose.pose().toPose2d();
							return pose;
						} else {
							// fallback to current pose if coral lost
							return drivetrainS.getLookAheadPose();
						}
					};
					PathConstraints constraints = new PathConstraints(
							DriveConstants.kMaxSpeedMetersPerSecond,
							DriveConstants.maxTranslationalAcceleration.get(),
							DriveConstants.kMaxTurningSpeedRadPerSec,
							DriveConstants.maxRotationalAcceleration.get());
					activeAimCommand = new AimToRotation(coralPoseSupplier, ApproachDirection.FRONT_RIGHT, drivetrainS,
							constraints);
					// initialize it
					try {
						activeAimCommand.initialize();
						aimInitialized = true;
					} catch (Exception e) {
						activeAimCommand = null;
						aimInitialized = false;
						System.out.println("DrivetrainC failed to initialize AimToRotation: " + e.toString());
					}
				}
		}
		if (!DriveConstants.autoIntake && lastAutoIntake) {
			if (activeAimCommand != null && aimInitialized) {
				try {
					activeAimCommand.end(true);
				} catch (Exception e) {
					System.out.println("DrivetrainC error ending AimToRotation: " + e.toString());
				}
			}
			activeAimCommand = null;
			aimInitialized = false;
		}

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
		// record last state for next tick
		lastAutoIntake = DriveConstants.autoIntake;

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
				Logger.recordOutput("Controller/SetY", ySpeed);
				drivetrainS.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));// for odom
				drivetrainS.stopModules();
			} else {
				// Deal with opposing robots.
				if (DriveConstants.autoAvoidance) {
					chassisSpeeds = avoidRobots(chassisSpeeds);
				}
				// Deal with coral
				if (DriveConstants.autoIntake && drivetrainS instanceof Swerve) {
					TxTyPoseRecord coralRec = ((Swerve) drivetrainS).getClosestCoralPose();
					Pose2d ourPose = drivetrainS.getLookAheadPose();
					if (isRecentValidCoral(coralRec) && ourPose != null) {
						Pose3d coralP3 = coralRec.pose();
						double dx = coralP3.getTranslation().getX() - ourPose.getX();
						double dy = coralP3.getTranslation().getY() - ourPose.getY();
						double dist = Math.hypot(dx, dy);
						if (dist < 1e-6) return;
						Rotation2d robotRot = drivetrainS.getRotation2d();
						double cos = Math.cos(-robotRot.getRadians());
						double sin = Math.sin(-robotRot.getRadians());

						double rx = (cos * dx - sin * dy) / dist; 

						double driverVx = chassisSpeeds.vxMetersPerSecond;

						double driverMag = Math.hypot(
								chassisSpeeds.vxMetersPerSecond,
								chassisSpeeds.vyMetersPerSecond
						);

						double desiredVx = Math.signum(rx) * driverMag;

						double k = autoIntakeAssistPercentage.get(); // 0–1 where 0.5 = can go ZERO speed "towards" coral, but cannot INCREASE distance from coral
						//ANYTHING above .5 means that the input *must* move towards coral, cannot go away
						//ANYTHING below .5 means that the input *does not have to* move towards coral, can go away if desired
						double correctedVx = MathUtil.interpolate(driverVx, desiredVx, k);

						chassisSpeeds = new ChassisSpeeds(
								correctedVx,
								chassisSpeeds.vyMetersPerSecond,
								chassisSpeeds.omegaRadiansPerSecond
						);
					} else {
						if (activeAimCommand != null && aimInitialized) {
							try {
								activeAimCommand.end(true);
							} catch (Exception e) {}
							activeAimCommand = null;
							aimInitialized = false;
						}
					}
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

		drivetrainS.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
