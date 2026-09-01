// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

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
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.Nat;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.MecanumDriveKinematics;
import org.wpilib.math.kinematics.MecanumDriveWheelPositions;
import org.wpilib.math.kinematics.MecanumDriveWheelVelocities;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.Position;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.vision.VisionConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Mecanum extends SubsystemChecker implements DrivetrainS {
	public static final double WHEEL_RADIUS = DriveConstants.TrainConstants.kWheelDiameter.get()
			/ 2;

	public enum DriveMode {
		NORMAL, WHEEL_RADIUS_CHARACTERIZATION, SPEED_CHARACTERIZATION
	}

	private record NextMotorOutput(MecanumDriveWheelVelocities wheelSpeeds, double[] voltages) {
	}

	private NextMotorOutput nextMotorOutput = new NextMotorOutput(new MecanumDriveWheelVelocities(), new double[4]);
	private DriveMode currentDriveMode = DriveMode.NORMAL;
	private double characterizationVelocity = 0.0;
	public static final double TRACK_WIDTH = DriveConstants.kChassisWidth;
	private final MecanumIO io;
	private final MecanumIOInputsAutoLogged inputs = new MecanumIOInputsAutoLogged();
	private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
			DriveConstants.kModuleTranslations[0],
			DriveConstants.kModuleTranslations[1],
			DriveConstants.kModuleTranslations[2],
			DriveConstants.kModuleTranslations[3]);
	private final SimpleMotorFeedforward feedforward = DriveConstants.overallDriveMotorConstantContainer
			.getFeedforward();
	private final double poseBufferSizeSeconds = 2;
	private Twist2d fieldVelocity;
	private Position<MecanumDriveWheelPositions> wheelPositions;
	private boolean collisionDetected;
	private Rotation2d rawGyroRotation = new Rotation2d();
	private int debounce = 0;
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
			.createBuffer(poseBufferSizeSeconds);

	public record VisionObservation(Pose2d visionPose, double timestamp,
			Matrix<N3, N1> stdDevs) {
	}

	public record OdometryObservation(MecanumDriveWheelPositions wheelPositions,
			Rotation2d gyroAngle, double timestamp) {
	}

	private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
	private Rotation2d lastGyroAngle = new Rotation2d();
	private MecanumDriveWheelPositions lastPositions = null;
	private Pose2d odometryPose = new Pose2d();
	private Pose2d estimatedPose = new Pose2d();

	/** Creates a new Drive. */
	public Mecanum(MecanumIO io) {
		this.io = io;
		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(this::getPose, this::resetPose,
				this::getChassisVelocities, this::setPathplannerChassisVelocities,
				DriveConstants.mainController,
				DriveConstants.mainConfig,
				() -> Robot.isRed, this);

		for (int i = 0; i < 3; ++i) {
			qStdDevs.set(i, 0, Math.pow(
					DriveConstants.TrainConstants.odometryStateStdDevs.get(i, 0),
					2));
		}
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("RobotState/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("RobotState/TrajectorySetpoint", targetPose);
		});
		registerSelfCheckHardware();
	}

	@Override
	public ChassisVelocities getChassisVelocities() {
		return kinematics.toChassisVelocities(
				new MecanumDriveWheelVelocities(getFrontLeftVelocityMetersPerSec(),
						getFrontRightVelocityMetersPerSec(),
						getBackLeftVelocityMetersPerSec(),
						getBackRightVelocityMetersPerSec()));
	}
	@Override
	public ChassisVelocities getFieldChassisVelocities(){
		return frc.robot.utils.drive.ChassisVelocityUtil.fromRobotRelative(getChassisVelocities(), getRotation2d());
	}

	@Override
	public void setChassisVelocities(ChassisVelocities speeds) {
		currentDriveMode = DriveMode.NORMAL;
		pathplannerIndex = 0;
		MecanumDriveWheelVelocities wheelSpeeds = kinematics.toWheelVelocities(speeds);
		driveVelocity(wheelSpeeds, false);
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
		odometryPose = odometryPose.plus(twist.exp());
		// Add pose to buffer at timestamp
		poseBuffer.addSample(observation.timestamp(), odometryPose);
		// Calculate diff from last odometry pose and add onto pose estimate
		estimatedPose = estimatedPose.plus(twist.exp());
	}

	public void addVisionObservation(VisionObservation observation) {
		try {
			if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds > observation.timestamp()) {
				return;
			}
		} catch (NoSuchElementException ex) {
			return;
		}
		var sample = poseBuffer.getSample(observation.timestamp());
		if (sample.isEmpty()) {
			return;
		}
		var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
		var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
		Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
		var r = new double[3];
		for (int i = 0; i < 3; ++i) {
			r[i] = Math.pow(observation.stdDevs().get(i, 0), 2);
		}
		Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
		for (int row = 0; row < 3; ++row) {
			double variance = qStdDevs.get(row, 0);
			if (row == 2 && RobotContainer.shouldReduceGyroYawTrust()) {
				double scale = Math.max(1e-3,
						Math.min(1.0, VisionConstants.shootingGyroYawTrustScale.get()));
				variance /= scale;
			}
			visionK.set(row, row,
					variance == 0.0 ? 0.0 : variance / (variance + Math.sqrt(variance * r[row])));
		}
		Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
		var correction = visionK.times(VecBuilder.fill(
				transform.getX(), transform.getY(), transform.getRotation().getRadians()));
		Transform2d scaledTransform = new Transform2d(
				correction.get(0, 0), correction.get(1, 0), Rotation2d.fromRadians(correction.get(2, 0)));
		estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
	}

	@Override
	public void periodic() {
		long timestampNs = System.nanoTime();
		io.updateInputs(inputs);
		Logger.processInputs("Mecanum", inputs);
		Logger.recordOutput("SystemStatus/Periodic/DriveInputsMS", (System.nanoTime() - timestampNs) / 1.0e6);
		timestampNs = System.nanoTime();
		// Update odometry
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		if (debounce == 1 && isConnected()) {
			resetPose(getPose());
			debounce = 0;
		}
		ChassisVelocities m_ChassisVelocities = getChassisVelocities();
		Logger.recordOutput("Mecanum/ChassisVelocities", m_ChassisVelocities);
		if (inputs.gyroConnected) {
			// Use the real gyro angle
			rawGyroRotation = inputs.gyroYaw;
		} else {
			rawGyroRotation = rawGyroRotation.plus(
					new Rotation2d(m_ChassisVelocities.omega * .004));
		}
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisVelocities.vx,
				m_ChassisVelocities.vy).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisVelocities.omega);
		addOdometryObservation(new OdometryObservation(wheelPositions.getPositions(),
				rawGyroRotation, wheelPositions.getTimestamp()));
		collisionDetected = collisionDetected();
		switch (currentDriveMode) {
			case WHEEL_RADIUS_CHARACTERIZATION:
				ChassisVelocities speeds = new ChassisVelocities(0, 0, characterizationVelocity);
				driveVelocity(kinematics.toWheelVelocities(speeds), true);
				break;
			case SPEED_CHARACTERIZATION:
				driveVolts(characterizationVelocity, characterizationVelocity,
						characterizationVelocity, characterizationVelocity);
				break;
			case NORMAL:
				io.setVelocity(nextMotorOutput.wheelSpeeds.frontLeft,
						nextMotorOutput.wheelSpeeds.frontRight,
						nextMotorOutput.wheelSpeeds.rearLeft,
						nextMotorOutput.wheelSpeeds.rearRight,
						nextMotorOutput.voltages[0], nextMotorOutput.voltages[1],
						nextMotorOutput.voltages[2], nextMotorOutput.voltages[3]);
				break;
		}
		DrivetrainS.super.periodic();
		Logger.recordOutput("SystemStatus/Periodic/DriveProcessMS", (System.nanoTime() - timestampNs) / 1.0e6);
	}

	/** Run open loop at the specified voltage. */
	public void driveVolts(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		io.setVoltage(frontLeftVolts, frontRightVolts, backLeftVolts,
				backRightVolts);
	}

	int pathplannerIndex = 0;
	boolean movingRight = false;

	@Override
	/**
	 * Run closed loop given speeds + feedforwards, sends feedforward volt to motor
	 */
	public void setPathplannerChassisVelocities(ChassisVelocities speeds, DriveFeedforwards feedforwards) {
		currentDriveMode = DriveMode.NORMAL;
		MecanumDriveWheelVelocities wheelSpeeds = kinematics.toWheelVelocities(speeds);
		// make the wheel speeds into a list, FL FR BL BR
		double[] wheelRadSpeedsArray = { wheelSpeeds.frontLeft / WHEEL_RADIUS,
				wheelSpeeds.frontRight / WHEEL_RADIUS,
				wheelSpeeds.rearLeft / WHEEL_RADIUS,
				wheelSpeeds.rearRight / WHEEL_RADIUS };
		// for loop, getting each motor NM, then converting to volts
		double[] feedForwardVolts = new double[4];
		pathplannerIndex++;
		for (int i = 0; i < 4; i++) {
			// only the robot relative x and y forces are provided for Choreo.
			double xForce = feedforwards.robotRelativeForcesXNewtons()[i];
			double yForce = feedforwards.robotRelativeForcesYNewtons()[i];
			if (pathplannerIndex == 1) {
				double angle = Math.atan2(yForce, xForce);
				if (angle > -Math.PI / 2 && angle < Math.PI / 2) {
					movingRight = true;
				} else {
					movingRight = false;
				}
			}
			double linearForce = Math.sqrt(xForce * xForce + yForce * yForce);
			double velocityMagnitude = Math.hypot(speeds.vx, speeds.vy);

			// Calculate the dot product to determine if force aligns with velocity
			double dotProduct = (xForce * speeds.vx + yForce * speeds.vy);

			// Sign adjustment based on alignment with velocity direction
			double signAdjustment = Math.signum(dotProduct / (linearForce * velocityMagnitude));
			if (Double.isNaN(signAdjustment)) {
				signAdjustment = 1;
			}
			// Assign the adjusted force magnitude
			double nmVal = linearForce * signAdjustment * (movingRight ? 1 : -1)
					* DriveConstants.TrainConstants.kWheelDiameter.get() / 2;
			double speedVoltage = wheelRadSpeedsArray[i] / DriveConstants.getDriveTrainMotors(1).Kv; // Voltage
																														// from
																														// speed
			double resistanceVoltage = nmVal / DriveConstants.getDriveTrainMotors(1).Kt
					* DriveConstants.getDriveTrainMotors(1).R; // Voltage
			// Total voltage required considering both speed and resistance
			feedForwardVolts[i] = resistanceVoltage + speedVoltage;
		}
		// send the volts to the motors
		nextMotorOutput = new NextMotorOutput(wheelSpeeds, feedForwardVolts);
	}

	/** Run closed loop at the specified voltage. */
	public void driveVelocity(MecanumDriveWheelVelocities wheelSpeeds, boolean setSpeeds) {
		currentDriveMode = DriveMode.NORMAL;
		double frontLeftRadPerSec = wheelSpeeds.frontLeft
				/ WHEEL_RADIUS;
		double frontRightRadPerSec = wheelSpeeds.frontRight
				/ WHEEL_RADIUS;
		double backLeftRadPerSec = wheelSpeeds.rearLeft
				/ WHEEL_RADIUS;
		double backRightRadPerSec = wheelSpeeds.rearRight
				/ WHEEL_RADIUS;
		nextMotorOutput = new NextMotorOutput(wheelSpeeds, new double[] {
				feedforward.calculate(getFrontLeftVelocityMetersPerSec() /WHEEL_RADIUS,
						frontLeftRadPerSec),
				feedforward.calculate(getFrontRightVelocityMetersPerSec() / WHEEL_RADIUS,
						frontRightRadPerSec),
				feedforward.calculate(getBackLeftVelocityMetersPerSec() /WHEEL_RADIUS,
						backLeftRadPerSec),
				feedforward.calculate(getBackRightVelocityMetersPerSec() /WHEEL_RADIUS,
						backRightRadPerSec)
		});
		if (setSpeeds) {
			io.setVelocity(frontLeftRadPerSec, frontRightRadPerSec, backLeftRadPerSec,
					backRightRadPerSec, nextMotorOutput.voltages[0], nextMotorOutput.voltages[1],
					nextMotorOutput.voltages[2], nextMotorOutput.voltages[3]);
		}
	}

	/** Stops the drive. */
	@Override
	public void stopModules() {
		currentDriveMode = DriveMode.NORMAL;
		driveVelocity(new MecanumDriveWheelVelocities(), true);
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

	/** Returns the current odometry pose in meters. */
	@AutoLogOutput(key = "RobotState/EstimatedPose")
	@Override
	public Pose2d getPose() {
		return estimatedPose;
	}
	@Override
	public Pose2d getLookAheadPose() {
		return estimatedPose.plus(getChassisVelocities().toTwist2d(.05).exp());
	}
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

	@Override
	public double[] getWheelRadiusCharacterizationPosition() {
		return new double[] { inputs.leftFrontPositionRad, inputs.rightFrontPositionRad,
				inputs.leftBackPositionRad, inputs.rightBackPositionRad };
	}

	/** Returns the average velocity in radians/second. */
	@Override
	@AutoLogOutput(key = "RobotState/Velocity")
	public double getCharacterizationVelocity() {
		ChassisVelocities chassisSpeeds = getChassisVelocities();
		return Math.sqrt(Math.pow(chassisSpeeds.vx, 2) + Math.pow(chassisSpeeds.vy, 2) + Math.pow(getChassisVelocities().omega * WHEEL_RADIUS, 2));
	
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
				run(() -> setChassisVelocities(new ChassisVelocities(0, 0, 0.5)))
						.withTimeout(2.0),
				run(() -> setChassisVelocities(new ChassisVelocities(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisVelocities(new ChassisVelocities(1, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					if (getChassisVelocities().vx > 1.2
							|| getChassisVelocities().vx < .8) {
						addFault(
								"[System Check] Forward speed did not reah target speed in time.",
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisVelocities(new ChassisVelocities(0, 0, 0))));
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
