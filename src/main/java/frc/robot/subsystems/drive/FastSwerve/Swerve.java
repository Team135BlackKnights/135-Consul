package frc.robot.subsystems.drive.FastSwerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.FastSwerve.Setpoints.SwerveSetpointGenerator;
import frc.robot.subsystems.drive.FastSwerve.Setpoints.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.Sensors.GyroIO;
import frc.robot.utils.drive.Sensors.GyroIOInputsAutoLogged;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemChecker implements DrivetrainS {
	//This is used
	@SuppressWarnings("unused")
	private static final LoggableTunedNumber coastWaitTime = new LoggableTunedNumber(
			"Drive/CoastWaitTimeSeconds", 0.5);
	private static final LoggableTunedNumber coastMetersPerSecThreshold = new LoggableTunedNumber(
			"Drive/CoastMetersPerSecThreshold", 0.25);

	public enum DriveMode {
		/** Driving with input from driver joysticks. (Default) */
		TELEOP,
		/** Driving based on a trajectory. */
		TRAJECTORY,
		/** Driving to a location on the field automatically. */
		AUTO_ALIGN,
	}

	public enum CoastRequest {
		AUTOMATIC, ALWAYS_BRAKE, ALWAYS_COAST
	}

	public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(
			20);
	private final OdometryThreadInputsAutoLogged odometryTimestampInputs;
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4];
	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			DriveConstants.kModuleTranslations);
	// Store previous positions and time for filtering odometry data
	private SwerveDriveWheelPositions lastPositions = null;
	private double lastTime = 0.0;
	/** Active drive mode. */
	private DriveMode currentDriveMode = DriveMode.TELEOP;
	private boolean modulesOrienting = false;
	private final Timer lastMovementTimer = new Timer();
	@AutoLogOutput(key = "Drive/BrakeModeEnabled")
	private boolean brakeModeEnabled = false;
	@AutoLogOutput(key = "Drive/CoastRequest")
	private CoastRequest coastRequest = CoastRequest.AUTOMATIC;
	private boolean lastEnabled = false;
	private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
	private static final double poseBufferSizeSeconds = 1.0;
	private Pose2d odometryPose = new Pose2d();
	private Pose2d estimatedPose = new Pose2d();
	private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
			new ChassisSpeeds(),
			new SwerveModuleState[] { new SwerveModuleState(),
					new SwerveModuleState(), new SwerveModuleState(),
					new SwerveModuleState()
			});
	private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer
			.createBuffer(poseBufferSizeSeconds);

	public record OdometryObservation(SwerveDriveWheelPositions wheelPositions,
			Rotation2d gyroAngle, double timestamp) {}

	private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

	public record ModuleLimits(double maxDriveVelocity,
			double maxDriveAcceleration, double maxSteeringVelocity) {}

	public record VisionObservation(Pose2d visionPose, double timestamp,
			Matrix<N3, N1> stdDevs) {}

	private final SysIdRoutine sysId;
	private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(
			new SwerveModulePosition[] { new SwerveModulePosition(),
					new SwerveModulePosition(), new SwerveModulePosition(),
					new SwerveModulePosition()
			});
	private Rotation2d lastGyroAngle = new Rotation2d();
	private Twist2d robotVelocity = new Twist2d();
	private final SwerveSetpointGenerator setpointGenerator;
	private boolean collisionDetected;
	boolean[] isSkidding = new boolean[] { false, false, false, false
	};
	private final OdometryThread odometryThread;

	public Swerve(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl,
			ModuleIO br) {
		this.gyroIO = gyroIO;
		modules[0] = new Module(fl, 0);
		modules[1] = new Module(fr, 1);
		modules[2] = new Module(bl, 2);
		modules[3] = new Module(br, 3);
		lastMovementTimer.start();
		setBrakeMode(true);
		for (int i = 0; i < 3; ++i) {
			qStdDevs.set(i, 0, Math.pow(
					DriveConstants.TrainConstants.odometryStateStdDevs.get(i, 0),
					2));
		}
		setpointGenerator = new SwerveSetpointGenerator(kinematics,
				DriveConstants.kModuleTranslations);
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
						new ReplanningConfig(true, true), .02),
				() -> Robot.isRed, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput("Drive/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> {
					for (int i = 0; i < 4; i++) {
						modules[i].runCharacterization(0, voltage.in(Volts));
					}
				}, null, this));
		setBrakeMode(true);
		registerSelfCheckHardware();
		this.odometryThread = OdometryThread.createInstance();
		this.odometryTimestampInputs = new OdometryThreadInputsAutoLogged();
		this.odometryThread.start();
	}

	/** Add odometry observation */
	public void addOdometryObservation(OdometryObservation observation) {
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions,
				observation.wheelPositions());
		lastWheelPositions = observation.wheelPositions();
		// Check gyro connected
		if (observation.gyroAngle != null) {
			// Update dtheta for twist if gyro connected
			twist = new Twist2d(twist.dx, twist.dy,
					observation.gyroAngle().minus(lastGyroAngle).getRadians());
			lastGyroAngle = observation.gyroAngle();
		} else {
			twist = new Twist2d(twist.dx, twist.dy, twist.dtheta);
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
				System.out.println("OUTSIDE BUFFER");
				return;
			}
		}
		catch (NoSuchElementException ex) {
			System.err.println("NO ELEMENT!");
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

	public void addVelocityData(Twist2d robotVelocity) {
		this.robotVelocity = robotVelocity;
	}

	@Override
	public SwerveDriveKinematics getKinematics() { return kinematics; }

	public boolean[] calculateSkidding() {
		SwerveModuleState[] moduleStates = getModuleStates();
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
		// Step 1: Create a measured ChassisSpeeds object with solely the rotation component
		ChassisSpeeds rotationOnlySpeeds = new ChassisSpeeds(0.0, 0.0,
				currentChassisSpeeds.omegaRadiansPerSecond + .05);
		double[] xComponentList = new double[4];
		double[] yComponentList = new double[4];
		// Step 2: Convert it into module states with kinematics
		SwerveModuleState[] rotationalStates = kinematics
				.toSwerveModuleStates(rotationOnlySpeeds);
		// Step 3: Subtract the rotational states from the module states to get the translational vectors and calculate the magnitudes.
		// These should all be the same direction and magnitude if there is no skid. 
		for (int i = 0; i < moduleStates.length; i++) {
			double deltaX = moduleStates[i].speedMetersPerSecond
					* Math.cos(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.cos(rotationalStates[i].angle.getRadians());
			double deltaY = moduleStates[i].speedMetersPerSecond
					* Math.sin(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.sin(rotationalStates[i].angle.getRadians());
			xComponentList[i] = deltaX;
			yComponentList[i] = deltaY;
		}
		//Step 4: Compare all of the translation vectors. If they aren't the same, skid is present.
		Arrays.sort(xComponentList);
		Arrays.sort(yComponentList);
		double deltaMedianX = (xComponentList[1] + xComponentList[2]) / 2;
		double deltaMedianY = (yComponentList[1] + yComponentList[2]) / 2;
		boolean[] areModulesSkidding = new boolean[4];
		for (int i = 0; i < 4; i++) {
			double deltaX = xComponentList[i];
			double deltaY = yComponentList[i];
			if (Math.abs(deltaX - deltaMedianX) > DriveConstants.SKID_THRESHOLD
					|| Math.abs(
							deltaY - deltaMedianY) > DriveConstants.SKID_THRESHOLD) {
				areModulesSkidding[i] = true;
			} else {
				areModulesSkidding[i] = false;
			}
		}
		Logger.recordOutput("Drive/Skids", areModulesSkidding);
		return areModulesSkidding;
	}

	/**
	 * Reset estimated pose and odometry pose to pose <br>
	 * Clear pose buffer
	 */
	public void resetPose(Pose2d initialPose) {
		estimatedPose = initialPose;
		odometryPose = initialPose;
		poseBuffer.clear();
	}

	@AutoLogOutput(key = "RobotState/FieldVelocity")
	@Override
	public Twist2d getFieldVelocity() {
		Translation2d linearFieldVelocity = new Translation2d(robotVelocity.dx,
				robotVelocity.dy).rotateBy(estimatedPose.getRotation());
		return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(),
				robotVelocity.dtheta);
	}

	/**
	 * Get the current pose of the robot with front being whatever front has been
	 * set to
	 * 
	 * @return a VISUAL ONLY output of the robot pose
	 * @see {@link #getPose() getPose} for the geometrically accurate pose
	 */
	@AutoLogOutput(key = "RobotState/EstimatedPose")
	public Pose2d getEstimatedPose() {
		return estimatedPose.plus(new Transform2d(new Translation2d(),
				DriveConstants.TrainConstants.robotOffsetAngleDirection));
	}

	public void periodic() {
		//Check if modules are skidding
		isSkidding = calculateSkidding();
		// Update & process inputs
		odometryThread.lockOdometry();
		odometryThread.updateInputs(odometryTimestampInputs);
		// Read timestamps from odometry thread and fake sim timestamps
		odometryTimestampInputs.measurementTimeStamps = timestampQueue.stream()
				.mapToDouble(Double::valueOf).toArray();
		if (odometryTimestampInputs.measurementTimeStamps.length == 0) { //for sim
			odometryTimestampInputs.measurementTimeStamps = new double[] {
					Timer.getFPGATimestamp()
			};
		}
		timestampQueue.clear();
		Logger.processInputs("Drive/OdometryTimestamps", odometryTimestampInputs);
		// Read inputs from gyro
		gyroIO.updateInputs(gyroInputs);
		gyroInputs.yawPosition = gyroInputs.yawPosition
				.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		for (Rotation2d value : gyroInputs.odometryYawPositions) {
			value.plus(DriveConstants.TrainConstants.robotOffsetAngleDirection);
		}
		Logger.processInputs("Drive/Gyro", gyroInputs);
		// Read inputs from modules
		Arrays.stream(modules).forEach(Module::updateInputs);
		odometryThread.unlockOdometry();
		ModuleLimits currentModuleLimits = DriveConstants.moduleLimitsFree; //implement limiting based off what you need
		// Calculate the min odometry position updates across all modules
		int minOdometryUpdates = IntStream
				.of(odometryTimestampInputs.measurementTimeStamps.length,
						Arrays.stream(modules)
								.mapToInt(module -> module.getModulePositions().length)
								.min().orElse(0))
				.min().orElse(0);
		if (gyroInputs.connected) {
			minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length,
					minOdometryUpdates);
		}
		// Pass odometry data to robot state
		for (int i = 0; i < minOdometryUpdates; i++) {
			int odometryIndex = i;
			Rotation2d yaw = gyroInputs.connected
					? gyroInputs.odometryYawPositions[i]
					: null;
			// Get all four swerve module positions at that odometry update
			// and store in SwerveDriveWheelPositions object
			SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(
					Arrays.stream(modules)
							.map(module -> module.getModulePositions()[odometryIndex])
							.toArray(SwerveModulePosition[]::new));
			// Filtering based on delta wheel positions
			boolean includeMeasurement = true;
			if (lastPositions != null) {
				double dt = odometryTimestampInputs.measurementTimeStamps[i] - lastTime;
				for (int j = 0; j < modules.length; j++) {
					double velocity = (wheelPositions.positions[j].distanceMeters
							- lastPositions.positions[j].distanceMeters) / dt;
					double omega = wheelPositions.positions[j].angle
							.minus(lastPositions.positions[j].angle).getRadians() / dt;
					// Check if delta is too large
					if (Math.abs(omega) > currentModuleLimits.maxSteeringVelocity()
							* 5.0
							|| Math.abs(velocity) > currentModuleLimits
									.maxDriveVelocity() * 5.0) {
						includeMeasurement = false;
						break;
					}
				}
			}
			// If delta isn't too large we can include the measurement.
			if (includeMeasurement) {
				lastPositions = wheelPositions;
				addOdometryObservation(new OdometryObservation(wheelPositions, yaw,
						odometryTimestampInputs.measurementTimeStamps[i]));
				lastTime = odometryTimestampInputs.measurementTimeStamps[i];
			}
		}
		// Update current velocities use gyro when possible
		ChassisSpeeds robotRelativeVelocity = getChassisSpeeds();
		robotRelativeVelocity.omegaRadiansPerSecond = gyroInputs.connected
				? gyroInputs.yawVelocityRadPerSec
				: robotRelativeVelocity.omegaRadiansPerSecond;
		addVelocityData(GeomUtil.toTwist2d(robotRelativeVelocity));
		// Update brake mode
		// Reset movement timer if moved
		if (Arrays.stream(modules)
				.anyMatch(module -> Math.abs(
						module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold
								.get())) {
			lastMovementTimer.reset();
		}
		if (DriverStation.isEnabled() && !lastEnabled) {
			coastRequest = CoastRequest.AUTOMATIC;
		}
		lastEnabled = DriverStation.isEnabled();
		switch (coastRequest) {
		case AUTOMATIC -> {
			if (DriverStation.isEnabled()) {
				setBrakeMode(true);
			} else {
				setBrakeMode(false);
			}
		}
		case ALWAYS_BRAKE -> {
			setBrakeMode(true);
		}
		case ALWAYS_COAST -> {
			setBrakeMode(false);
		}
		}
		// Run modules
		if (!modulesOrienting) {
			// Run robot at desiredSpeeds
			// Generate feasible next setpoint
			SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
			SwerveModuleState[] optimizedSetpointTorques = new SwerveModuleState[4];
			currentSetpoint = setpointGenerator.generateSetpoint(
					currentModuleLimits, currentSetpoint, desiredSpeeds, .02);
			for (int i = 0; i < modules.length; i++) {
				// Optimize setpoints
				optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];
				optimizedSetpointTorques[i] = new SwerveModuleState(0.0,
						optimizedSetpointStates[i].angle);
				modules[i].runSetpoint(optimizedSetpointStates[i],
						optimizedSetpointTorques[i]);
			}
			Logger.recordOutput("Drive/SwerveStates/Setpoints",
					optimizedSetpointStates);
			Logger.recordOutput("Drive/SwerveStates/Torques",
					optimizedSetpointTorques);
		}
		Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
		Logger.recordOutput("Drive/SetpointSpeeds",
				currentSetpoint.chassisSpeeds());
		Logger.recordOutput("Drive/DriveMode", currentDriveMode);
		collisionDetected = collisionDetected();
		DrivetrainS.super.periodic();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		desiredSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
		desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);
	}

	@Override
	public void setDiscreteChassisSpeeds(ChassisSpeeds speeds) {
		desiredSpeeds = new ChassisSpeeds(speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
	}

	/**
	 * Set brake mode to {@code enabled} doesn't change brake mode if already
	 * set.
	 */
	private void setBrakeMode(boolean enabled) {
		if (brakeModeEnabled != enabled) {
			Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
		}
		brakeModeEnabled = enabled;
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of
	 * the modules.
	 */
	@AutoLogOutput(key = "Drive/SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		return Arrays.stream(modules).map(Module::getState)
				.toArray(SwerveModuleState[]::new);
	}

	/**
	 * Returns the measured speeds of the robot in the robot's frame of
	 * reference.
	 */
	@AutoLogOutput(key = "Drive/MeasuredSpeeds")
	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp", modules[0].getDriveMotorTemp());
		tempMap.put("FLTurnTemp", modules[0].getTurnMotorTemp());
		tempMap.put("FRDriveTemp", modules[1].getDriveMotorTemp());
		tempMap.put("FRTurnTemp", modules[1].getTurnMotorTemp());
		tempMap.put("BLDriveTemp", modules[2].getDriveMotorTemp());
		tempMap.put("BLTurnTemp", modules[2].getTurnMotorTemp());
		tempMap.put("BRDriveTemp", modules[3].getDriveMotorTemp());
		tempMap.put("BRTurnTemp", modules[3].getTurnMotorTemp());
		return tempMap;
	}

	private void registerSelfCheckHardware() {
		//super.registerAllHardware(gyroIO.getSelfCheckingHardware());
		super.registerAllHardware(modules[0].getSelfCheckingHardware());
		super.registerAllHardware(modules[1].getSelfCheckingHardware());
		super.registerAllHardware(modules[2].getSelfCheckingHardware());
		super.registerAllHardware(modules[3].getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		for (Module module : modules) {
			List<SelfChecking> moduleHardware = module.getSelfCheckingHardware();
			for (SelfChecking motor : moduleHardware) {
				if (motor.getHardware() instanceof TalonFX) {
					orchestra.add((TalonFX) motor.getHardware());
				}
			}
		}
		return orchestra;
	}

	public boolean[] isSkidding() { return isSkidding; }

	@Override
	public double getCurrent() {
		return modules[0].getCurrent() + modules[1].getCurrent()
				+ modules[2].getCurrent() + modules[3].getCurrent();
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
		return Commands
				.sequence(run(() -> setChassisSpeeds(new ChassisSpeeds(1, 0, 0)))
						.withTimeout(1.0), runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								// Retrieve the corresponding REVSwerveModule from the hashmap
								// Get the name of the current ModulePosition
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < .8
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.2) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow (wanted 1) for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 0) >= 10
										&& Math.abs(Math.abs(angle) - 180) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1, 0)))
								.withTimeout(1.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < .8
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.2) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow (wanted 1) for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 90) >= 10
										&& Math.abs(Math.abs(angle) - 270) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ String.format("%.2f", angle),
											false, true);
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -2)))
								.withTimeout(2.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								double angle = moduleState.angle.getDegrees();
								if (i == 0) {
									name = "Front Left";
									if (Math.abs(Math.abs(angle) - 135) >= 10
											&& Math.abs(Math.abs(angle) - 315) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 1) {
									name = "Front Right";
									if (Math.abs(Math.abs(angle) - 45) >= 10
											&& Math.abs(Math.abs(angle) - 225) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 2) {
									name = "Back Left";
									if (Math.abs(Math.abs(angle) - 135) >= 10
											&& Math.abs(Math.abs(angle) - 315) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 3) {
									name = "Back Right";
									if (Math.abs(Math.abs(angle) - 45) >= 10
											&& Math.abs(Math.abs(angle) - 225) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
							}
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 2)))
								.withTimeout(2.0),
						runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								double angle = moduleState.angle.getDegrees();
								if (i == 0) {
									name = "Front Left";
									if (Math.abs(Math.abs(angle) - 135) >= 10
											&& Math.abs(Math.abs(angle) - 315) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 1) {
									name = "Front Right";
									if (Math.abs(Math.abs(angle) - 45) >= 10
											&& Math.abs(Math.abs(angle) - 225) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 2) {
									name = "Back Left";
									if (Math.abs(Math.abs(angle) - 135) >= 10
											&& Math.abs(Math.abs(angle) - 315) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
								if (i == 3) {
									name = "Back Right";
									if (Math.abs(Math.abs(angle) - 45) >= 10
											&& Math.abs(Math.abs(angle) - 225) >= 10) {
										addFault(
												"[System Check] Turn angle off for " + name
														+ String.format("%.2f", angle),
												false, true);
									}
								}
							}
						}))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public void zeroHeading() { gyroIO.reset(); }

	@Override
	public boolean isConnected() { return gyroInputs.connected; }

	private boolean collisionDetected() { return gyroInputs.collisionDetected; }

	@Override
	public boolean isCollisionDetected() { return collisionDetected; }

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		addVisionObservation(new VisionObservation(pose, timestamp, estStdDevs));
	}

	/**
	 * Get the current pose of the robot with front being the GEOMETRY front
	 * 
	 * @return an INTERNAL ONLY output of the robot pose (use this for any
	 *         driving/turning calculations)
	 * @see {@link #getEstimatedPose() getEstimatedPose} for the visually
	 *      accurate pose
	 */
	@Override
	public Pose2d getPose() { return estimatedPose; }

	@Override
	public void stopModules() { setChassisSpeeds(new ChassisSpeeds()); }

	@Override
	public Rotation2d getRotation2d() { return getPose().getRotation(); }

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

	@Override
	public void setDriveCurrentLimit(int amps) {
		Arrays.stream(modules).forEach(module -> module.setCurrentLimit(amps));
	}

	@Override
	public void setCurrentLimit(int amps) { setDriveCurrentLimit(amps); }
}
