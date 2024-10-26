package frc.robot.utils.CompetitionFieldUtils.Simulation;

/*import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Mecanum.Mecanum;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.FastSwerve.OdometryThread;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim;
import frc.robot.subsystems.drive.Mecanum.MecanumIOSim.MecanumDrivePhysicsSimResults;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.AbstractDriveTrainSimulation;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.RobotPhysicsSimulationConfigs;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.Sensors.GyroIOSim;
import frc.robot.utils.maths.GeometryConvertor;

import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;
import frc.robot.utils.maths.CommonMath;
import java.util.function.Consumer;*/

/**
 * TODO Simulates the dynamics of a mecanum robot. Uses essentially the same code as
 * a swerve drive sim (physics included). Takes motor behaviors from ModuleIOSim
 * and feeds the values back as simulated "encoder readings"
 */
/*
public class MecanumDriveSimulation extends AbstractDriveTrainSimulation {
	private final  Mecanum mecanum;
	private final  MecanumIOSim mecanumIOSim;
	private final GyroIOSim gyroIOSim;
	private final MecanumDriveKinematics kinematics;
	private final Consumer<Pose2d> resetOdometryCallBack;
	private double gForce = 0.0; //G
	private ChassisSpeeds desiredFieldRelativeSpeeds = new ChassisSpeeds();

	public double convertRadPerSecondtoMeterPerSecond(double radPerSecond) {
		return radPerSecond * TrainConstants.kDriveMotorGearRatio
				* TrainConstants.kWheelDiameter / 2;
	}

	public MecanumDriveSimulation(DriveTrainSimulationProfile robotProfile, GyroIOSim gyroIOSim,
			MecanumDriveKinematics kinematics, Pose2d startingPose,
			Mecanum mecanum, MecanumIOSim ioSim,Consumer<Pose2d> resetOdometryCallBack) {
    	super(robotProfile, startingPose, resetOdometryCallBack);
		this.gyroIOSim = gyroIOSim;
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
		mecanum.updateSim(RobotPhysicsSimulationConfigs.SIMULATION_DT);
		//should do the actual motion calculations
		simulateChassisBehaviorWithFieldRelativeSpeeds(desiredFieldRelativeSpeeds);
		updateGyroSimulationResults(gyroIOSim,
				getObjectOnFieldPose2d().getRotation(),
				super.getAngularVelocity(), gForce, iterationNum);
		updateMecanumSimulationResults(mecanum, mecanumIOSim, actualModuleFloorSpeeds,
				profile.robotMaxVelocity, iterationNum, RobotPhysicsSimulationConfigs.SIMULATION_DT,
				instantVelocityRobotRelative);
	}
	private void simulateChassisRotationalBehavior(double desiredRotationalMotionPercent) {
		final double maximumTorque = this.profile.maxAngularAcceleration * super.getMass().getInertia();
		super.applyTorque(desiredRotationalMotionPercent * maximumTorque);
		simulateChassisAngularFriction(desiredRotationalMotionPercent);
	 }
private void simulateChassisBehaviorWithFieldRelativeSpeeds(
      ChassisSpeeds desiredChassisSpeedsFieldRelative) {
    super.setAtRest(false);

    final Vector2 desiredLinearMotionPercent =
        GeometryConvertor.toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
            .multiply(1.0 / profile.maxLinearVelocity);
    simulateChassisTranslationalBehavior(
        Vector2.create(
            CommonMath.constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1),
            desiredLinearMotionPercent.getDirection()));

    final double desiredRotationalMotionPercent =
        desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity;
    simulateChassisRotationalBehavior(CommonMath.constrainMagnitude(desiredRotationalMotionPercent, 1));
  }
	 private void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
    final boolean robotRequestedToMoveLinearly = desiredLinearMotionPercent.getMagnitude() > 0.03;
    final Vector2 forceVec =
        desiredLinearMotionPercent
            .copy()
            .multiply(this.profile.robotMass * this.profile.maxLinearAcceleration);

    if (robotRequestedToMoveLinearly) super.applyForce(new Force(forceVec));
    else simulateChassisLinearFriction();
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

	private static void updateMecanumSimulationResults( Mecanum mecanum, MecanumIOSim mecanumIOSim,
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
		//Convert mecanum array into wheel speeds (the loop should always iterate 4 times)
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
		//Motor efficiency? I.E. how much of the free speed does it get at max speed? 
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
*/