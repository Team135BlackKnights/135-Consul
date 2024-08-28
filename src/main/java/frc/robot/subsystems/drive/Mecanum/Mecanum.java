// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.NoSuchElementException;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.Position;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Mecanum extends SubsystemChecker implements DrivetrainS {
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter
			/ 2;
	public static final double TRACK_WIDTH = DriveConstants.kChassisWidth;
	private final MecanumIO io;
	private final MecanumIOInputsAutoLogged inputs = new MecanumIOInputsAutoLogged();
	private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
			DriveConstants.kModuleTranslations[0],
			DriveConstants.kModuleTranslations[1],
			DriveConstants.kModuleTranslations[2],
			DriveConstants.kModuleTranslations[3]);
	private final SimpleMotorFeedforward feedforward = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getFeedforward();
	private final SysIdRoutine sysId;
	private final double poseBufferSizeSeconds = 2;
	private Twist2d fieldVelocity;
	private Position<MecanumDriveWheelPositions> wheelPositions;
	private boolean collisionDetected;
	private Rotation2d rawGyroRotation = new Rotation2d();
	private int debounce = 0;
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
			.createBuffer(poseBufferSizeSeconds);

	public record VisionObservation(Pose2d visionPose, double timestamp,
			Matrix<N3, N1> stdDevs) {}

	public record OdometryObservation(MecanumDriveWheelPositions wheelPositions,
			Rotation2d gyroAngle, double timestamp) {}

	private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
	private Rotation2d lastGyroAngle = new Rotation2d();
	private MecanumDriveWheelPositions lastPositions = null;
	private Pose2d odometryPose = new Pose2d();
	private Pose2d estimatedPose = new Pose2d();
	/** Creates a new Drive. */
	public Mecanum(MecanumIO io) {
		this.io = io;
		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configureHolonomic(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds,
				new HolonomicPathFollowerConfig(new PIDConstants(
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getP(),
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getI(),
						DriveConstants.TrainConstants.pathplannerTranslationConstantContainer
								.getD()),
						new PIDConstants(
								DriveConstants.TrainConstants.pathplannerRotationConstantContainer
										.getP(),
								DriveConstants.TrainConstants.pathplannerRotationConstantContainer
										.getI(),
								DriveConstants.TrainConstants.pathplannerRotationConstantContainer
										.getD()),
						DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.kDriveBaseRadius,
						new ReplanningConfig(true, true)),
				() -> Robot.isRed, this);

		for (int i = 0; i < 3; ++i) {
					qStdDevs.set(i, 0, Math.pow(
							DriveConstants.TrainConstants.odometryStateStdDevs.get(i, 0),
							2));
				}
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		// Configure SysId
		Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
		Measure<Voltage> holdVoltage = Volts.of(4);
		Measure<Time> timeout = Seconds.of(10);
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
						(state) -> Logger.recordOutput("Mecanum/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism(
						(voltage) -> driveVolts(voltage.in(Volts), voltage.in(Volts),
								voltage.in(Volts), voltage.in(Volts)),
						null, this));
		registerSelfCheckHardware();
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(
				new MecanumDriveWheelSpeeds(getFrontLeftVelocityMetersPerSec(),
						getFrontRightVelocityMetersPerSec(),
						getBackLeftVelocityMetersPerSec(),
						getBackRightVelocityMetersPerSec()));
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		driveVelocity(wheelSpeeds);
	}

	private MecanumDriveWheelPositions getWheelPositions() {
		return new MecanumDriveWheelPositions(getFrontLeftPositionMeters(),
				getFrontRightPositionMeters(), getBackLeftPositionMeters(),
				getBackRightPositionMeters());
	}

	/** SIM ONLY */
	public void updateSim(double dtSeconds) {
		io.updateSim(dtSeconds);
	}
/** Add odometry observation */
	public void addOdometryObservation(OdometryObservation observation) {
		if (lastPositions == null) {
			lastPositions = new MecanumDriveWheelPositions();
			return;
		}
		Twist2d twist = kinematics.toTwist2d(lastPositions,
				observation.wheelPositions());
				lastPositions = observation.wheelPositions();
		// Check gyro connected
		if (observation.gyroAngle != null) {
			// Update dtheta for twist if gyro connected
			twist = new Twist2d(twist.dx, twist.dy,
					observation.gyroAngle().minus(lastGyroAngle).getRadians());
			lastGyroAngle = observation.gyroAngle();
		}
		// Add twist to odometry pose
		odometryPose = odometryPose.exp(twist);
		// Add pose to buffer at timestamp
		poseBuffer.addSample(observation.timestamp(), odometryPose);
		// Calculate diff from last odometry pose and add onto pose estimate
		estimatedPose = estimatedPose.exp(twist);
	}

	public void addVisionObservation(VisionObservation observation) {
		// If measurement is old enough to be outside the pose buffer's timespan, skip.
		try {
			if (poseBuffer.getInternalBuffer().lastKey()
					- poseBufferSizeSeconds > observation.timestamp()) {
				return;
			}
		}
		catch (NoSuchElementException ex) {
			return;
		}
		// Get odometry based pose at timestamp
		var sample = poseBuffer.getSample(observation.timestamp());
		if (sample.isEmpty()) {
			// exit if not there
			return;
		}
		// sample --> odometryPose transform and backwards of that
		var sampleToOdometryTransform = new Transform2d(sample.get(),
				odometryPose);
		var odometryToSampleTransform = new Transform2d(odometryPose,
				sample.get());
		// get old estimate by applying odometryToSample Transform
		Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
		// Calculate 3 x 3 vision matrix
		var r = new double[3];
		for (int i = 0; i < 3; ++i) {
			r[i] = observation.stdDevs().get(i, 0)
					* observation.stdDevs().get(i, 0);
		}
		// Solve for closed form Kalman gain for continuous Kalman filter with A = 0
		// and C = I. See wpimath/algorithms.md.
		Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < 3; ++row) {
			double stdDev = qStdDevs.get(row, 0);
			if (stdDev == 0.0) {
				visionK.set(row, row, 0.0);
			} else {
				visionK.set(row, row,
						stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
			}
		}
		// difference between estimate and vision pose
		Transform2d transform = new Transform2d(estimateAtTime,
				observation.visionPose());
		// scale transform by visionK
		var kTimesTransform = visionK.times(VecBuilder.fill(transform.getX(),
				transform.getY(), transform.getRotation().getRadians()));
		Transform2d scaledTransform = new Transform2d(kTimesTransform.get(0, 0),
				kTimesTransform.get(1, 0),
				Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
		// Recalculate current estimate by applying scaled transform to old estimate
		// then replaying odometry data
		estimatedPose = estimateAtTime.plus(scaledTransform)
				.plus(sampleToOdometryTransform);
	}
	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Mecanum", inputs);
		// Update odometry
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		if (debounce == 1 && isConnected()) {
			resetPose(getPose());
			debounce = 0;
		}
		ChassisSpeeds m_ChassisSpeeds = getChassisSpeeds();
		Logger.recordOutput("Mecanum/ChassisSpeeds", m_ChassisSpeeds);
		if (inputs.gyroConnected) {
			// Use the real gyro angle
			rawGyroRotation = inputs.gyroYaw;
		} else {
			rawGyroRotation = rawGyroRotation.plus(
					new Rotation2d(m_ChassisSpeeds.omegaRadiansPerSecond * .004));
		}
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
		addOdometryObservation(new OdometryObservation(wheelPositions.getPositions(),
				rawGyroRotation, wheelPositions.getTimestamp()));
		collisionDetected = collisionDetected();
		DrivetrainS.super.periodic();
	}

	/** Run open loop at the specified voltage. */
	public void driveVolts(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		io.setVoltage(frontLeftVolts, frontRightVolts, backLeftVolts,
				backRightVolts);
	}

	/** Run closed loop at the specified voltage. */
	public void driveVelocity(MecanumDriveWheelSpeeds wheelSpeeds) {
		double frontLeftRadPerSec = wheelSpeeds.frontLeftMetersPerSecond
				/ WHEEL_RADIUS;
		double frontRightRadPerSec = wheelSpeeds.frontRightMetersPerSecond
				/ WHEEL_RADIUS;
		double backLeftRadPerSec = wheelSpeeds.rearLeftMetersPerSecond
				/ WHEEL_RADIUS;
		double backRightRadPerSec = wheelSpeeds.rearRightMetersPerSecond
				/ WHEEL_RADIUS;
		io.setVelocity(frontLeftRadPerSec, frontRightRadPerSec, backLeftRadPerSec,
				backRightRadPerSec, feedforward.calculate(frontLeftRadPerSec),
				feedforward.calculate(frontRightRadPerSec),
				feedforward.calculate(backLeftRadPerSec),
				feedforward.calculate(backRightRadPerSec));
	}

	/** Stops the drive. */
	@Override
	public void stopModules() { driveVelocity(new MecanumDriveWheelSpeeds()); }

	/**
	 * Returns a command to run a quasistatic test in the specified direction.
	 */
	@Override
	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	@Override
	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}

	/** Returns the current odometry pose in meters. */
	@AutoLogOutput(key = "RobotState/EstimatedPose")
	@Override
	public Pose2d getPose() { return estimatedPose; }

	/** Resets the current odometry pose. */
	@Override
	public void resetPose(Pose2d pose) {
		estimatedPose = pose;
		odometryPose = pose;
		poseBuffer.clear();
	}

	/** Returns the position of the front left wheel in meters. */
	@AutoLogOutput
	public double getFrontLeftPositionMeters() {
		return inputs.leftFrontPositionRad * WHEEL_RADIUS;
	}

	/** Returns the position of the front right wheel in meters. */
	@AutoLogOutput
	public double getFrontRightPositionMeters() {
		return inputs.rightFrontPositionRad * WHEEL_RADIUS;
	}

	/** Returns the position of the back left wheel in meters. */
	@AutoLogOutput
	public double getBackLeftPositionMeters() {
		return inputs.leftBackPositionRad * WHEEL_RADIUS;
	}

	/** Returns the position of the back right wheel in meters. */
	@AutoLogOutput
	public double getBackRightPositionMeters() {
		return inputs.rightBackPositionRad * WHEEL_RADIUS;
	}

	/** Returns the velocity of the front left wheel in meters/second. */
	@AutoLogOutput
	public double getFrontLeftVelocityMetersPerSec() {
		return inputs.leftFrontVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the velocity of the front right wheel in meters/second. */
	@AutoLogOutput
	public double getFrontRightVelocityMetersPerSec() {
		return inputs.rightFrontVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the velocity of the back left wheel in meters/second. */
	@AutoLogOutput
	public double getBackLeftVelocityMetersPerSec() {
		return inputs.leftBackVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the velocity of the back right wheel in meters/second. */
	@AutoLogOutput
	public double getBackRightVelocityMetersPerSec() {
		return inputs.rightBackVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the average velocity in radians/second. */
	public double getCharacterizationVelocity() {
		return (inputs.leftFrontVelocityRadPerSec
				+ inputs.rightFrontVelocityRadPerSec
				+ inputs.leftBackVelocityRadPerSec
				+ inputs.rightBackVelocityRadPerSec) / 4.0;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(io.getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		List<SelfChecking> driveHardware = io.getSelfCheckingHardware();
		for (SelfChecking motor : driveHardware) {
			if (motor.getHardware() instanceof TalonFX) {
				orchestra.add((TalonFX) motor.getHardware());
			}
		}
		return orchestra;
	}

	@Override
	public double getCurrent() {
		return Math.abs(inputs.leftCurrentAmps[0])
				+ Math.abs(inputs.leftCurrentAmps[1])
				+ Math.abs(inputs.rightCurrentAmps[0])
				+ Math.abs(inputs.rightCurrentAmps[1]);
	}

	@Override
	public SystemStatus getTrueSystemStatus() { return getSystemStatus(); }

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(1, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vxMetersPerSecond > 1.2
							|| getChassisSpeeds().vxMetersPerSecond < .8) {
						addFault(
								"[System Check] Forward speed did not reah target speed in time.",
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		addVisionObservation(new VisionObservation(pose, timestamp, estStdDevs));
	}

	@Override
	public Rotation2d getRotation2d() { return rawGyroRotation; }

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@AutoLogOutput(key = "RobotState/FieldVelocity")
	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }

	@Override
	public void zeroHeading() {
		io.reset();
		debounce = 1;
	}

	@Override
	public boolean isConnected() { return inputs.gyroConnected; }

	private boolean collisionDetected() { return inputs.collisionDetected; }

	@Override
	public boolean isCollisionDetected() { return collisionDetected; }

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp", inputs.frontLeftDriveTemp);
		tempMap.put("FRDriveTemp", inputs.frontRightDriveTemp);
		tempMap.put("BLDriveTemp", inputs.backLeftDriveTemp);
		tempMap.put("BRDriveTemp", inputs.backRightDriveTemp);
		return tempMap;
	}

	@Override
	public void setDriveCurrentLimit(int amps) { io.setCurrentLimit(amps); }

	@Override
	public void setCurrentLimit(int amps) { setDriveCurrentLimit(amps); }
}
