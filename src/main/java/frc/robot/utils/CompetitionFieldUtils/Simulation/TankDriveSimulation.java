package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.subsystems.drive.Tank.TankIOSim;
import frc.robot.subsystems.drive.Tank.TankIOSim.TankDrivePhysicsSimResults;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.Sensors.GyroIOSim;
import frc.robot.utils.maths.GeometryConvertor;

import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

/**
 * Simulates the dynamics of a tank robot. This class is meant to be used in
 * simulation only.
 */
public class TankDriveSimulation extends HolonomicChassisSimulation {
	private final Tank tank;
	private final TankIOSim tankIOSim;
	private final GyroIOSim gyroIOSim;
	private final DifferentialDriveKinematics kinematics;
	private final Consumer<Pose2d> resetOdometryCallBack;
	private double gForce = 0.0; //G

	public double convertRadPerSecondtoMeterPerSecond(double radPerSecond) {
		return radPerSecond * TrainConstants.kDriveMotorGearRatio
				* TrainConstants.kWheelDiameter / 2;
	}

	public TankDriveSimulation(RobotProfile robotProfile, GyroIOSim gyroIOSim,
			DifferentialDriveKinematics kinematics, Pose2d startingPose, Tank tank,
			TankIOSim ioSim, Consumer<Pose2d> resetOdometryCallBack) {
		super(robotProfile, startingPose);
		this.gyroIOSim = gyroIOSim;
		this.tank = tank;
		this.tankIOSim = ioSim;
		this.kinematics = kinematics;
		this.resetOdometryCallBack = resetOdometryCallBack;
		resetOdometryToActualRobotPose();
	}

	@Override
	public void resetOdometryToActualRobotPose() {
		resetOdometryCallBack.accept(getObjectOnFieldPose2d());
	}

	@Override
	public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
		ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
				givenSpeeds.vxMetersPerSecond, 0,
				givenSpeeds.omegaRadiansPerSecond);
		super.setLinearVelocity(
				GeometryConvertor.toDyn4jLinearVelocity(adjustedSpeeds));
		super.setAngularVelocity(adjustedSpeeds.omegaRadiansPerSecond);
	}

	@Override
	public void simulateChassisBehaviorWithRobotRelativeSpeeds(
			ChassisSpeeds desiredChassisSpeedsRobotRelative) {
		ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
				desiredChassisSpeedsRobotRelative.vxMetersPerSecond, 0,
				desiredChassisSpeedsRobotRelative.omegaRadiansPerSecond);
		simulateChassisBehaviorWithFieldRelativeSpeeds(
				ChassisSpeeds.fromRobotRelativeSpeeds(adjustedSpeeds,
						getObjectOnFieldPose2d().getRotation()));
	}

	@Override
	public void updateSimulationSubPeriod(int iterationNum,
			double subPeriodSeconds) {
		tank.updateSim(subPeriodSeconds);
		//should do the actual motion calculations
		final ChassisSpeeds tankTheoreticalSpeeds = kinematics
				.toChassisSpeeds(tankIOSim.getWheelSpeeds());
		super.simulateChassisBehaviorWithRobotRelativeSpeeds(
				tankTheoreticalSpeeds);
		final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
		final DifferentialDriveWheelSpeeds actualModuleFloorSpeeds = kinematics
				.toWheelSpeeds(instantVelocityRobotRelative);
		updateGyroSimulationResults(gyroIOSim,
				super.getObjectOnFieldPose2d().getRotation(),
				super.getAngularVelocity(), gForce, iterationNum);
		updateTankSimulationResults(tank, tankIOSim, actualModuleFloorSpeeds,
				profile.robotMaxVelocity, iterationNum, subPeriodSeconds,
				instantVelocityRobotRelative);
	}

	private final Vector2 previousDesiredLinearMotionPercent = new Vector2();

	@Override
	protected void simulateChassisTranslationalBehavior(
			Vector2 desiredLinearMotionPercent) {
		super.simulateChassisTranslationalBehavior(desiredLinearMotionPercent);
		final double dTheta = previousDesiredLinearMotionPercent
				.getAngleBetween(desiredLinearMotionPercent),
				desiredMotionDirectionChangingRateOmega = dTheta
						/ (Robot.defaultPeriodSecs
								/ DriveConstants.RobotPhysicsSimulationConfigs.SIM_ITERATIONS_PER_ROBOT_PERIOD),
				centripetalForce = desiredMotionDirectionChangingRateOmega
						* getLinearVelocity().getMagnitude() * profile.robotMass;
		super.applyForce(Vector2.create(
				MathUtil.clamp(centripetalForce, -profile.frictionForce,
						profile.frictionForce),
				getLinearVelocity().getDirection() + Math.toRadians(90)));
		//calculate gForce
		gForce = Math
				.sqrt(Math.pow(getLinearVelocity().x, 2)
						+ Math.pow(getLinearVelocity().y, 2))
				/ DriveConstants.RobotPhysicsSimulationConfigs.FLOOR_FRICTION_ACCELERATION_METERS_PER_SEC_SQ;
		previousDesiredLinearMotionPercent.set(desiredLinearMotionPercent);
	}

	private static void updateGyroSimulationResults(GyroIOSim gyroIOSim,
			Rotation2d currentFacing, double angularVelocityRadPerSec,
			double gForce, int iterationNum) {
		final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
		results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
		results.gForce = gForce;
		results.odometryYawPositions[iterationNum] = currentFacing;
		results.hasReading = true;
	}

	private static void updateTankSimulationResults(Tank tank,
			TankIOSim tankIOSim, DifferentialDriveWheelSpeeds speeds,
			double robotMaxVelocity, int simulationIteration, double periodSeconds,
			ChassisSpeeds instantSpeed) {
		double[] freeWheelSpeeds = { tank.getLeftVelocityMetersPerSec(),
				tank.getRightVelocityMetersPerSec()
		};
		double[] degreeAngles = new double[2];
		double[] physicsAccurateWheelSpeeds = { speeds.leftMetersPerSecond,
				speeds.rightMetersPerSecond
		};
		final TankDrivePhysicsSimResults results = tankIOSim.tankDrivePhysicsSimResults;
		//Convert mecanum array into wheel speeds (the loop should always iterate 4 times)
		for (int i = 0; i < freeWheelSpeeds.length; i++) {
			degreeAngles[i] = 0;
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
		//Motor efficiency? I.E. how much of the free speed does it get at max speed? 
		final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = .8,
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
				/ DriveConstants.TrainConstants.kWheelDiameter / 2;
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
