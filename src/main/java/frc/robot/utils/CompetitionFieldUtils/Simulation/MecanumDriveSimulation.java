package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim.MecanumDrivePhysicsSimResults;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.GyroSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.SimplifiedHolonomicDriveSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Consumer;

/**
 * Simulates the dynamics of a mecanum robot. Uses essentially the same code as
 * a swerve drive sim (physics included). Takes motor behaviors from ModuleIOSim
 * and feeds the values back as simulated "encoder readings"
 */

public class MecanumDriveSimulation extends SimplifiedHolonomicDriveSimulation {
	private int iterationNum = 0;
	private final Mecanum mecanum;
	private final MecanumIOSim mecanumIOSim;
	private final GyroSimulation gyroSim;
	private final double subPeriodSeconds = Robot.defaultPeriodSecs
			/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
	private final MecanumDriveKinematics kinematics;
	private final Consumer<Pose2d> resetOdometryCallBack;

	public double convertRadPerSecondtoMeterPerSecond(double radPerSecond) {
		return radPerSecond * TrainConstants.kDriveMotorGearRatioLow
				* TrainConstants.kWheelDiameter.get() / 2;
	}

	public MecanumDriveSimulation(DriveTrainSimulationProfile robotProfile, GyroSimulation gyroSim,
			MecanumDriveKinematics kinematics, Pose2d startingPose,
			Mecanum mecanum, MecanumIOSim ioSim, Consumer<Pose2d> resetOdometryCallBack) {
		super(robotProfile, startingPose, resetOdometryCallBack);
		this.gyroSim = gyroSim;
		this.mecanum = mecanum;
		this.mecanumIOSim = ioSim;
		this.kinematics = kinematics;
		this.resetOdometryCallBack = resetOdometryCallBack;

		resetOdometryToActualRobotPose();
	}

	@Override
	public void resetOdometryToActualRobotPose() {
		resetOdometryCallBack.accept(getObjectOnFieldPose2d());
	}

	@Override
	public void simulationSubTick() {

		mecanum.updateSim(subPeriodSeconds);
		// should do the actual motion calculations
		final ChassisSpeeds mecanumTheoreticalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(kinematics
		.toChassisSpeeds(mecanumIOSim.getWheelSpeeds()),getObjectOnFieldPose2d().getRotation().unaryMinus());
		super.simulateChassisBehaviorWithFieldRelativeSpeeds(
				mecanumTheoreticalSpeeds);
		final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
		final MecanumDriveWheelSpeeds actualModuleFloorSpeeds = kinematics
				.toWheelSpeeds(instantVelocityRobotRelative);
		gyroSim.updateSimulationSubTick(angularVelocity);
		updateMecanumSimulationResults(mecanum, mecanumIOSim, actualModuleFloorSpeeds,
				profile.maxLinearVelocity, iterationNum, subPeriodSeconds,
				instantVelocityRobotRelative);
		iterationNum++;
		iterationNum %= DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;

	}

	private static void updateMecanumSimulationResults(Mecanum mecanum, MecanumIOSim mecanumIOSim,
			MecanumDriveWheelSpeeds speeds, double robotMaxVelocity,
			int simulationIteration, double periodSeconds,
			ChassisSpeeds instantSpeed) {
		double[] freeWheelSpeeds = { mecanum.getFrontLeftVelocityMetersPerSec(),
				mecanum.getFrontRightVelocityMetersPerSec(),
				mecanum.getBackLeftVelocityMetersPerSec(),
				mecanum.getBackRightVelocityMetersPerSec()
		};
		double[] degreeAngles = new double[4];
		double[] physicsAccurateWheelSpeeds = { speeds.frontLeftMetersPerSecond,
				speeds.frontRightMetersPerSecond, speeds.rearLeftMetersPerSecond,
				speeds.rearRightMetersPerSecond
		};
		final MecanumDrivePhysicsSimResults results = mecanumIOSim.mecanumDrivePhysicsSimResults;
		// Convert mecanum array into wheel speeds (the loop should always iterate 4
		// times)
		for (int i = 0; i < freeWheelSpeeds.length; i++) {
			degreeAngles[i] = (TrainConstants.mecanumInitialAngleOffsetDegrees
					+ i * 90) % 360;
			results.driveWheelFinalVelocityRevolutionsPerSec[i] = getActualDriveMotorRotterSpeedRevPerSec(
					physicsAccurateWheelSpeeds[i], freeWheelSpeeds[i]);
			if ((Math.sqrt(Math.pow(instantSpeed.vxMetersPerSecond, 2)
					+ Math.pow(instantSpeed.vyMetersPerSecond, 2)) > 0.1)
					|| (Math.sqrt(
							Math.pow(instantSpeed.omegaRadiansPerSecond, 2)) > 0.05)) {
				results.negateFF[i] = false;
			} else {
				results.negateFF[i] = true;
			}
			results.driveWheelFinalRevolutions[i] += results.driveWheelFinalVelocityRevolutionsPerSec[i]
					* periodSeconds;
			results.odometryDriveWheelRevolutions[i][simulationIteration] = results.driveWheelFinalRevolutions[i];
		}
	}

	private static double getActualDriveMotorRotterSpeedRevPerSec(
			double moduleSpeedProjectedOnSwerveHeadingMPS,
			double moduleFreeSpeedMPS) {
		// Motor efficiency? I.E. how much of the free speed does it get at max speed?
		final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = 0.8,
				rotorSpeedMetersPerSecond;
		if (Math.abs(
				moduleFreeSpeedMPS - moduleSpeedProjectedOnSwerveHeadingMPS) < 2)
			rotorSpeedMetersPerSecond = moduleSpeedProjectedOnSwerveHeadingMPS;
		else
			rotorSpeedMetersPerSecond = moduleSpeedProjectedOnSwerveHeadingMPS
					* FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED
					+ moduleFreeSpeedMPS
							* (1 - FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED);
		final double rotorSpeedRadPerSec = rotorSpeedMetersPerSecond
				/ DriveConstants.TrainConstants.kWheelDiameter.get() / 2;
		return Units.radiansToRotations(rotorSpeedRadPerSec);
	}

	public static final class OdometryThreadSim implements OdometryThread {
		@Override
		public void updateInputs(OdometryThreadInputs inputs) {
			inputs.measurementTimeStamps = new double[DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD];
			final double robotStartingTimeStamps = Logger.getTimestamp(),
					iterationPeriodSeconds = Robot.defaultPeriodSecs
							/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD;
			for (int i = 0; i < DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD; i++)
				inputs.measurementTimeStamps[i] = robotStartingTimeStamps
						+ i * iterationPeriodSeconds;
		}
	}
}
