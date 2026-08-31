// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.NoSuchElementException;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.util.Nat;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.DifferentialDriveKinematics;
import org.wpilib.math.kinematics.DifferentialDriveWheelPositions;
import org.wpilib.math.kinematics.DifferentialDriveWheelVelocities;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.Position;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Tank extends SubsystemChecker implements DrivetrainS {
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter.get()
			/ 2;
	public static final double TRACK_WIDTH = DriveConstants.kChassisWidth;
	private final TankIO io;
	private final TankIOInputsAutoLogged inputs = new TankIOInputsAutoLogged();
	private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
			TRACK_WIDTH);
	private final SimpleMotorFeedforward feedforward = DriveConstants.overallDriveMotorConstantContainer
			.getFeedforward();
	private record NextMotorOutput(DifferentialDriveWheelVelocities wheelSpeeds, double[] voltages) {
			}
			public enum DriveMode {
				NORMAL, WHEEL_RADIUS_CHARACTERIZATION, SPEED_CHARACTERIZATION
			}
	private NextMotorOutput nextMotorOutput = new NextMotorOutput(new DifferentialDriveWheelVelocities(), new double[2]);
	private double characterizationVelocity = 0.0;
	private DriveMode currentDriveMode = DriveMode.NORMAL;

	private final double poseBufferSizeSeconds = 2;
	private Twist2d fieldVelocity;
	private Rotation2d rawGyroRotation = new Rotation2d();
	private Position<DifferentialDriveWheelPositions> wheelPositions;
	private boolean collisionDetected;
	private int debounce = 0;
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
			.createBuffer(poseBufferSizeSeconds);

	public record VisionObservation(Pose2d visionPose, double timestamp,
			Matrix<N3, N1> stdDevs) {
	}

	public record OdometryObservation(
			DifferentialDriveWheelPositions wheelPositions, Rotation2d gyroAngle,
			double timestamp) {
	}

	private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
	private Rotation2d lastGyroAngle = new Rotation2d();
	private DifferentialDriveWheelPositions lastPositions = null;
	private Pose2d odometryPose = new Pose2d();
	private Pose2d estimatedPose = new Pose2d();

	/** Creates a new Drive. */
	public Tank(TankIO io) {
		this.io = io;
		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setPathplannerChassisSpeeds, DriveConstants.mainController,
				DriveConstants.mainConfig, () -> Robot.isRed, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		for (int i = 0; i < 3; ++i) {
			qStdDevs.set(i, 0, Math.pow(
					DriveConstants.TrainConstants.odometryStateStdDevs.get(i, 0),
					2));
		}
		registerSelfCheckHardware();
	}

	/** Add odometry observation */
	public void addOdometryObservation(OdometryObservation observation) {
		if (lastPositions == null) {
			lastPositions = new DifferentialDriveWheelPositions(0, 0);
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
		odometryPose = odometryPose.plus(twist.exp());
		// Add pose to buffer at timestamp
		poseBuffer.addSample(observation.timestamp(), odometryPose);
		// Calculate diff from last odometry pose and add onto pose estimate
		estimatedPose = estimatedPose.plus(twist.exp());
	}

	public void addVisionObservation(VisionObservation observation) {
		// If measurement is old enough to be outside the pose buffer's timespan, skip.
		try {
			if (poseBuffer.getInternalBuffer().lastKey()
					- poseBufferSizeSeconds > observation.timestamp()) {
				return;
			}
		} catch (NoSuchElementException ex) {
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
	public ChassisVelocities getChassisSpeeds() {
		return kinematics.toChassisVelocities(new DifferentialDriveWheelVelocities(
				getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec()));
	}

	/** SIM ONLY */
	public void updateSim(double dtSeconds) {
		io.updateSim(dtSeconds);
	}

	@Override
	public void setChassisSpeeds(ChassisVelocities speeds) {
		currentDriveMode = DriveMode.NORMAL;
		DifferentialDriveWheelVelocities wheelSpeeds = kinematics
				.toWheelVelocities(speeds);
		driveVelocity(wheelSpeeds,false);
	}

	@Override
	public void setPathplannerChassisSpeeds(ChassisVelocities speeds, DriveFeedforwards feedforwards) {
		currentDriveMode = DriveMode.NORMAL;
		DifferentialDriveWheelVelocities wheelSpeeds = kinematics
				.toWheelVelocities(speeds);
		double leftFeedForwardVolts = ((wheelSpeeds.left / WHEEL_RADIUS)
						/ DriveConstants.getDriveTrainMotors(1).Kv);

		double rightFeedForwardVolts = ((wheelSpeeds.left / WHEEL_RADIUS)
						/ DriveConstants.getDriveTrainMotors(1).Kv);
		double leftResistanceVoltage = feedforwards.torqueCurrentsAmps()[0]
				* DriveConstants.getDriveTrainMotors(1).R;
		double rightResistanceVoltage = feedforwards.torqueCurrentsAmps()[2]
				* DriveConstants.getDriveTrainMotors(1).R;
		nextMotorOutput = new NextMotorOutput(wheelSpeeds, new double[]{leftFeedForwardVolts + leftResistanceVoltage, rightFeedForwardVolts + rightResistanceVoltage});
	}

	private DifferentialDriveWheelPositions getWheelPositions() {
		return new DifferentialDriveWheelPositions(getLeftPositionMeters(),
				getRightPositionMeters());
	}

	@Override
	public void periodic() {
		long timestamp = System.currentTimeMillis();
		io.updateInputs(inputs);
		Logger.processInputs("Drive", inputs);
		Logger.recordOutput("SystemStatus/Periodic/DriveInputsMS", System.currentTimeMillis() - timestamp);
		timestamp = System.currentTimeMillis();
		// Update odometry
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		if (debounce == 1 && isConnected()) {
			resetPose(estimatedPose);
			debounce = 0;
		}
		ChassisVelocities m_ChassisSpeeds = getChassisSpeeds();
		if (inputs.gyroConnected) {
			// Use the real gyro angle
			rawGyroRotation = inputs.gyroYaw;
		} else {
			rawGyroRotation = rawGyroRotation.plus(
					new Rotation2d(m_ChassisSpeeds.omega * .02));
		}
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vx,
				m_ChassisSpeeds.vy).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omega);
		addOdometryObservation(
				new OdometryObservation(wheelPositions.getPositions(),
						rawGyroRotation, wheelPositions.getTimestamp()));
		collisionDetected = collisionDetected();
		switch (currentDriveMode) {
			case WHEEL_RADIUS_CHARACTERIZATION:
				ChassisVelocities speeds = new ChassisVelocities(0, 0, characterizationVelocity);
				driveVelocity(kinematics.toWheelVelocities(speeds), true);
				break;
			case SPEED_CHARACTERIZATION:
				driveVolts(characterizationVelocity, characterizationVelocity);
				break;
			case NORMAL:
				io.setVelocity(nextMotorOutput.wheelSpeeds.left / WHEEL_RADIUS,
						nextMotorOutput.wheelSpeeds.right / WHEEL_RADIUS,
						nextMotorOutput.voltages[0], nextMotorOutput.voltages[1]);
				break;
		}
		DrivetrainS.super.periodic();
		Logger.recordOutput("SystemStatus/Periodic/DriveProcessMS", System.currentTimeMillis() - timestamp);

	}

	/** Run open loop at the specified voltage. */
	public void driveVolts(double leftVolts, double rightVolts) {
		io.setVoltage(leftVolts, rightVolts);
	}

	/** Run closed loop at the specified voltage. */
	public void driveVelocity(DifferentialDriveWheelVelocities wheelSpeeds, boolean setSpeeds) {
		double leftRadPerSec = wheelSpeeds.left / WHEEL_RADIUS;
		double rightRadsPerSec = wheelSpeeds.right / WHEEL_RADIUS;
		nextMotorOutput = new NextMotorOutput(wheelSpeeds, new double[]{feedforward.calculate(getLeftVelocityMetersPerSec() / WHEEL_RADIUS, leftRadPerSec),
			feedforward.calculate(getRightVelocityMetersPerSec() / WHEEL_RADIUS, rightRadsPerSec)});
		if (setSpeeds)
			io.setVelocity(leftRadPerSec, rightRadsPerSec,
					nextMotorOutput.voltages[0], nextMotorOutput.voltages[1]);
	}

	/** Stops the drive. */
	@Override
	public void stopModules() {
		driveVelocity(new DifferentialDriveWheelVelocities(),true);
	}

	/** Returns the current odometry pose in meters. */
	@AutoLogOutput(key = "RobotState/EstimatedPose")
	@Override
	public Pose2d getPose() {
		return estimatedPose;
	}
	@Override
	public Pose2d getLookAheadPose() {
		return estimatedPose.plus(getChassisSpeeds().toTwist2d(.02).exp());
	}

	/** Resets the current odometry pose. */
	@Override
	public void resetPose(Pose2d pose) {
		estimatedPose = pose;
		odometryPose = pose;
		poseBuffer.clear();
	}

	/** Returns the position of the left wheels in meters. */
	@AutoLogOutput
	public double getLeftPositionMeters() {
		return inputs.leftPositionRad * WHEEL_RADIUS;
	}

	/** Returns the position of the right wheels in meters. */
	@AutoLogOutput
	public double getRightPositionMeters() {
		return inputs.rightPositionRad * WHEEL_RADIUS;
	}

	/** Returns the velocity of the left wheels in meters/second. */
	@AutoLogOutput
	public double getLeftVelocityMetersPerSec() {
		return inputs.leftVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the velocity of the right wheels in meters/second. */
	@AutoLogOutput
	public double getRightVelocityMetersPerSec() {
		return inputs.rightVelocityRadPerSec * WHEEL_RADIUS;
	}

	/** Returns the average velocity in radians/second. */
	@Override
	@AutoLogOutput(key = "RobotState/Velocity")
	public double getCharacterizationVelocity() {
		ChassisVelocities chassisSpeeds = getChassisSpeeds();
		return Math.sqrt(Math.pow(chassisSpeeds.vx, 2) + Math.pow(chassisSpeeds.vy, 2) + Math.pow(getChassisSpeeds().omega * WHEEL_RADIUS, 2));
	}
	@Override 
	public double[] getWheelRadiusCharacterizationPosition(){
		return new double[] { inputs.leftPositionRad, inputs.rightPositionRad };
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
	public void runWheelRadiusCharacterization(double velocity) {
		currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
		characterizationVelocity = velocity;
	}

	@Override
	public void runCharacterization(double input) {
		currentDriveMode = DriveMode.SPEED_CHARACTERIZATION;
		characterizationVelocity = input;
	}
	@Override
	public void endCharacterization() {
		currentDriveMode = DriveMode.NORMAL;
	}
	@Override
	public double getCurrent() {
		if (inputs.leftCurrentAmps.length == 1) {
			return Math.abs(inputs.leftCurrentAmps[0])
					+ Math.abs(inputs.rightCurrentAmps[0]);
		}
		return Math.abs(inputs.leftCurrentAmps[0])
				+ Math.abs(inputs.leftCurrentAmps[1])
				+ Math.abs(inputs.rightCurrentAmps[0])
				+ Math.abs(inputs.rightCurrentAmps[1]);
	}

	@Override
	public SystemStatus getTrueSystemStatus() {
		return getSystemStatus();
	}

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
				run(() -> setChassisSpeeds(new ChassisVelocities(0, 0, 0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisVelocities(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisVelocities(1, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vx > 1.2
							|| getChassisSpeeds().vx < .8) {
						addFault(
								"[System Check] Forward speed did not reah target speed in time.",
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisVelocities(0, 0, 0))));
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		addVisionObservation(new VisionObservation(pose, timestamp, estStdDevs));
	}

	@Override
	public Rotation2d getRotation2d() {
		return rawGyroRotation;
	}

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; // ?
	}

	@AutoLogOutput(key = "RobotState/FieldVelocity")
	@Override
	public Twist2d getFieldVelocity() {
		return fieldVelocity;
	}

	/**
	 * UNTESTED!
	 */
	@Override
	public void zeroHeading() {
		io.reset();
		debounce = 1;
	}

	@Override
	public boolean isConnected() {
		return inputs.gyroConnected;
	}

	private boolean collisionDetected() {
		return inputs.collisionDetected;
	}

	@Override
	public boolean isCollisionDetected() {
		return collisionDetected;
	}

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
	public void setDriveCurrentLimit(int amps) {
		io.setCurrentLimit(amps);
	}

	@Override
	public void setCurrentLimit(int amps) {
		setDriveCurrentLimit(amps);
	}
}
