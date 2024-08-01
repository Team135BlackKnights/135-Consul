package frc.robot.utils.CompetitionFieldUtils.Simulation;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

import java.util.HashMap;

import org.ejml.simple.UnsupportedOperation;

/**
 * simulates an opponent robot on field in physics, the opponent robot behaves
 * just the same as our own robot, it also follows the Holonomic Chassis Physics
 * the difference is, opponent robots are not controlled by the main gamepad it
 * is either controlled by another gamepad to simulate a defense robot or can
 * follow pre-generated paths to simulate opponent robots who are doing cycles
 */
public class OpponentRobotSimulation extends HolonomicChassisSimulation
		implements DrivetrainS {
	public enum Behavior {
		JOYSTICK_CONTROL, AUTO_CYCLE, QUEEN
	}

	/* if an opponent robot is not requested to be on field, it queens outside the field for performance */
	public static final Pose2d robotQueeningPosition = new Pose2d(-5, 5,
			new Rotation2d());
	public static final HolonomicChassisSimulation.RobotProfile opponentRobotProfile = new RobotProfile(
			4, 12, Math.toRadians(360), Units.lbsToKilograms(125),
			DriveConstants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_WIDTH_METERS,
			DriveConstants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_LENGTH_METERS);
	private final int id;

	/**
	 * @param id the id of the robot, 0 to 2, this determines where the robot
	 *              "respawns"
	 */
	public OpponentRobotSimulation(int id) {
		super(opponentRobotProfile, robotQueeningPosition);
		if (id >= 3)
			throw new IllegalArgumentException("id must be 0~2");
		this.id = id;
	}

	private ChassisSpeeds speedSetPoint = new ChassisSpeeds();

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		this.speedSetPoint = speeds;
	}

	@Override
	public Pose2d getPose() { return super.getObjectOnFieldPose2d(); }

	@Override
	public void resetPose(Pose2d currentPose) {
		super.setSimulationWorldPose(currentPose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		throw new UnsupportedOperation(
				"an opponent robot does not support vision measurement");
	}

	@Override
	public void updateSimulationSubPeriod(int iterationNum,
			double subPeriodSeconds) {
		super.simulateChassisBehaviorWithRobotRelativeSpeeds(speedSetPoint);
	}

	public Command getAutoCyleCommand() {
		final PathPlannerPath cyclePathRaw = PathPlannerPath
				.fromPathFile("opponent cycle path " + id),
				cyclePath = Robot.isRed ? cyclePathRaw.flipPath() : cyclePathRaw,
				cyclePathReversed = FollowPathOpponent.reversePath(cyclePath,
						new GoalEndState(0, cyclePath
								.getPreviewStartingHolonomicPose().getRotation()));
		cyclePath.preventFlipping = cyclePathReversed.preventFlipping = true;
		final Command teleportToStartingPose = Commands
				.runOnce(() -> setSimulationWorldPose(
						cyclePathReversed.getPreviewStartingHolonomicPose())),
				cycleForward = new FollowPathOpponent(cyclePath, () -> false, this),
				cycleBackWards = new FollowPathOpponent(cyclePathReversed,
						() -> false, this);
		return new SequentialCommandGroup(teleportToStartingPose,
				new SequentialCommandGroup(cycleBackWards, cycleForward)
						.repeatedly())
								.finallyDo(() -> setChassisSpeeds(new ChassisSpeeds()));
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return ChassisSpeeds.fromFieldRelativeSpeeds(
				getMeasuredChassisSpeedsFieldRelative(),
				getObjectOnFieldPose2d().getRotation());
	}

	@Override
	public void stopModules() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'stopModules'");
	}

	@Override
	public Rotation2d getRotation2d() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getRotation2d'");
	}

	@Override
	public Twist2d getFieldVelocity() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getFieldVelocity'");
	}

	@Override
	public Command sysIdDynamicDrive(Direction kforward) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdDynamicDrive'");
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdQuasistaticDrive'");
	}

	@Override
	public void zeroHeading() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'zeroHeading'");
	}

	@Override
	public boolean isConnected() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'isConnected'");
	}

	@Override
	public boolean isCollisionDetected() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'isCollisionDetected'");
	}

	@Override
	public void setDriveCurrentLimit(int amps) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'setDriveCurrentLimit'");
	}

	@Override
	public HashMap<String, Double> getTemps() {
		throw new UnsupportedOperationException(
				"Unimplemented method 'getTemps'");
	}
}
