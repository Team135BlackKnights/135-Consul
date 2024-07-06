package frc.robot.subsystems.drive.OldREVSwerve;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.OldREVSwerve.SwerveModules.REVSwerveModule;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.HashMap;
import java.util.List;
import java.util.Arrays;
import java.util.Collections;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import frc.robot.utils.drive.Position;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class REVSwerveS extends SubsystemChecker implements DrivetrainS {
	private static Translation2d[] kModuleTranslations;
	private static double kMaxSpeedMetersPerSecond, kDriveBaseRadius;
	private static REVModuleConstantContainer[] REVModuleConstantContainers = new REVModuleConstantContainer[4];
	private static SwerveDriveKinematics kDriveKinematics;
	private HashMap<ModulePosition, REVSwerveModule> m_swerveModules = new HashMap<>();
	private static AHRS gyro = new AHRS(Port.kUSB1);
	//Whether the robot should be field oriented
	//Whether the swerve drivetrain should be taken over by our auto drive to game piece feature (in the Vision branch)
	public Pose2d robotPosition = new Pose2d(0, 0, getRotation2d());
	Field2d robotField = new Field2d();
	private Twist2d fieldVelocity = new Twist2d();
	// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
	ChassisSpeeds m_ChassisSpeeds;
	static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
	static Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);
	boolean[] isSkidding = new boolean[] { false, false, false, false
	};
	private boolean collisionDetected = false;
	public SwerveDrivePoseEstimator poseEstimator;
	Position<SwerveModulePosition[]> m_modulePositions;
	private double m_simYaw, last_world_linear_accel_x,
			last_world_linear_accel_y;;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(8);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (REVSwerveModule module : m_swerveModules.values())
					module.setTurningTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (REVSwerveModule module : m_swerveModules.values())
					module.setDriveTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));

	/**
	 * Creates a new REVSwerve Drivetrain with 8 CANSparkMaxes or 8
	 * CANSparkFlexes
	 * 
	 * @param maxSpeed                 the max speed of the drivetrain
	 * @param driveBaseRadius          the radius of the drivetrain
	 * @param moduleConstantContainers the moduleConstantContainers for each
	 *                                    module as an array of {frontLeft,
	 *                                    frontRight, backLeft, backRight}
	 * @see REVModuleConstantContainer
	 */
	public REVSwerveS(REVModuleConstantContainer[] moduleConstantContainers,
			double maxSpeed, double driveBaseRadius) {
		REVModuleConstantContainers = moduleConstantContainers;
		initalizeModules();
		kModuleTranslations = new Translation2d[] {
				moduleConstantContainers[0].getTranslation2d(),
				moduleConstantContainers[1].getTranslation2d(),
				moduleConstantContainers[2].getTranslation2d(),
				moduleConstantContainers[3].getTranslation2d()
		};
		kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
		m_ChassisSpeeds = kDriveKinematics.toChassisSpeeds(getModuleStates());
		m_modulePositions = getPositionsWithTimestamp(getModulePositions());
		poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics,
				getRotation2d(), m_modulePositions.getPositions(), robotPosition,
				stateStdDevs, visionStdDevs);
		kMaxSpeedMetersPerSecond = maxSpeed;
		kDriveBaseRadius = driveBaseRadius;
		// Waits for the RIO to finishing booting
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				zeroHeading();
				for (REVSwerveModule module : m_swerveModules.values())
					module.resetEncoders();
				//limelight.getEntry("pipeline").setNumber(1);
			}
			catch (Exception e) {
			}
		}).start();
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);
		registerSelfCheckHardware();
	}

	/**
	 * Calculates the translational vectors of each module, and confirms it is
	 * within .25 m/s of the median vector. If it isn't that module is said to be
	 * "Skidding"
	 * 
	 * @apiNote TEST ON BOT NEEDED
	 * @param m_ChassisSpeeds
	 * @return
	 */
	public boolean[] calculateSkidding() {
		SwerveModuleState[] moduleStates = getModuleStates();
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
		// Step 1: Create a ChassisSpeeds object with solely the rotation component
		ChassisSpeeds rotationOnlySpeeds = new ChassisSpeeds(0.0, 0.0,
				currentChassisSpeeds.omegaRadiansPerSecond + .05);
		double[] xComponentList = new double[4];
		double[] yComponentList = new double[4];
		// Step 2: Convert it into module states with kinematics
		SwerveModuleState[] rotationalStates = kDriveKinematics
				.toSwerveModuleStates(rotationOnlySpeeds);
		// Step 3: Subtract the rotational states from the module states and calculate the magnitudes
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
		Arrays.sort(xComponentList);
		Arrays.sort(yComponentList);
		SmartDashboard.putNumberArray("Module Skid X", xComponentList);
		SmartDashboard.putNumberArray("Module Skid Y", yComponentList);
		double deltaMedianX = (xComponentList[1] + xComponentList[2]) / 2;
		double deltaMedianY = (yComponentList[1] + yComponentList[2]) / 2;
		SmartDashboard.putNumber("Skid X Median", deltaMedianX);
		SmartDashboard.putNumber("Skid Y Median", deltaMedianY);
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
		SmartDashboard.putBooleanArray("Module Skids", areModulesSkidding);
		return areModulesSkidding;
	}

	@Override
	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.quasistatic(direction);
	}

	//public static boolean autoLock = false;
	//private static double kP, kI, kD, kDistanceMultipler;
	//public PIDController autoLockController; //sadly cannot be system Id'd
	//so that the navXDisconnect command doesn't start twice
	int debounce = 0;

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
		};
	}

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
		};
	}

	private void initalizeModules() {
		m_swerveModules.clear();
		REVSwerveModule front_left = new REVSwerveModule(
				REVModuleConstantContainers[0]);
		REVSwerveModule front_right = new REVSwerveModule(
				REVModuleConstantContainers[1]);
		REVSwerveModule back_left = new REVSwerveModule(
				REVModuleConstantContainers[2]);
		REVSwerveModule back_right = new REVSwerveModule(
				REVModuleConstantContainers[3]);
		for (ModulePosition position : ModulePosition.values()) {
			if (position.name() == "FRONT_LEFT") {
				m_swerveModules.put(position, front_left);
			} else if (position.name() == "FRONT_RIGHT") {
				m_swerveModules.put(position, front_right);
			} else if (position.name() == "BACK_LEFT") {
				m_swerveModules.put(position, back_left);
			} else if (position.name() == "BACK_RIGHT") {
				m_swerveModules.put(position, back_right);
			}
		}
		SmartDashboard.putData("Swerve Drive", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle",
						() -> m_swerveModules.get(ModulePosition.FRONT_LEFT)
								.getHeadingRotation2d().getRadians(),
						null);
				builder.addDoubleProperty(
						"Front Left Velocity", () -> m_swerveModules
								.get(ModulePosition.FRONT_LEFT).getDriveVelocity(),
						null);
				builder.addDoubleProperty("Front Right Angle",
						() -> m_swerveModules.get(ModulePosition.FRONT_RIGHT)
								.getHeadingRotation2d().getRadians(),
						null);
				builder.addDoubleProperty(
						"Front Right Velocity", () -> m_swerveModules
								.get(ModulePosition.FRONT_RIGHT).getDriveVelocity(),
						null);
				builder.addDoubleProperty("Back Left Angle",
						() -> m_swerveModules.get(ModulePosition.BACK_LEFT)
								.getHeadingRotation2d().getRadians(),
						null);
				builder.addDoubleProperty(
						"Back Left Velocity", () -> m_swerveModules
								.get(ModulePosition.BACK_LEFT).getDriveVelocity(),
						null);
				builder.addDoubleProperty("Back Right Angle",
						() -> m_swerveModules.get(ModulePosition.BACK_RIGHT)
								.getHeadingRotation2d().getRadians(),
						null);
				builder.addDoubleProperty(
						"Back Right Velocity", () -> m_swerveModules
								.get(ModulePosition.BACK_RIGHT).getDriveVelocity(),
						null);
				builder.addDoubleProperty("Robot Angle",
						() -> Units.degreesToRadians(getHeading()), null);
			}
		});
	}

	@Override
	public void zeroHeading() {
		debounce = 0;
		gyro.reset();
		poseEstimator.resetPosition(gyro.getRotation2d(),
				m_modulePositions.getPositions(), robotPosition);
	}

	public static double getHeading() {
		return -1 * Math.IEEEremainder(gyro.getAngle() + (Robot.isRed ? 180 : 0),
				360); //modulus
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}

	@Override
	public boolean[] isSkidding() { return isSkidding; }

	@Override
	public void periodic() {
		if (Constants.currentMode == Mode.SIM) {
			ChassisSpeeds chassisSpeed = kDriveKinematics
					.toChassisSpeeds(getModuleStates());
			m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
			//m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
		}
		//puts values to smartDashboard
		//SmartDashboard.putBoolean("Auto Lock", autoLock);
		SmartDashboard.putNumber("Robot Heading (getPose)",
				getPose().getRotation().getDegrees());
		m_modulePositions = getPositionsWithTimestamp(getModulePositions());
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		m_ChassisSpeeds = kDriveKinematics.toChassisSpeeds(getModuleStates());
		isSkidding = calculateSkidding();
		robotPosition = poseEstimator.updateWithTime(
				m_modulePositions.getTimestamp(), getRotation2d(),
				m_modulePositions.getPositions());
		for (REVSwerveModule module : m_swerveModules.values()) {
			module.updateModuleStates();
			var modulePositionFromChassis = kModuleTranslations[module
					.getModuleNumber()].rotateBy(getRotation2d())
							.plus(getPoseMeters().getTranslation());
			module.setModulePose(new Pose2d(modulePositionFromChassis,
					module.getHeadingRotation2d().plus(getRotation2d())));
		}
		DrivetrainS.super.periodic();
		navXDisconnectProtocol();
		Logger.recordOutput("Swerve/Display/Actual Swerve Module States",
				getModuleStates());
		Logger.recordOutput("Swerve/Display/Rotation",
				getRotation2d().getDegrees());
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
		boolean collisionDetected = collisionDetected();
		SmartDashboard.putBoolean("Collision Detected", collisionDetected);
	}

	@AutoLogOutput(key = "Odometry/Robot")
	@Override
	public Pose2d getPose() { return robotPosition; }

	@Override
	public ChassisSpeeds getChassisSpeeds() { return m_ChassisSpeeds; }

	@Override
	public void resetPose(Pose2d pose) {
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		poseEstimator.resetPosition(getRotation2d(),
				m_modulePositions.getPositions(), pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public void stopModules() {
		for (REVSwerveModule module : m_swerveModules.values())
			module.stop();
	}

	public Pose2d getPoseMeters() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speed) {
		SwerveModuleState[] moduleStates = kDriveKinematics
				.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
				kMaxSpeedMetersPerSecond);
		for (REVSwerveModule module : m_swerveModules.values()) {
			module.setDesiredState(moduleStates[module.getModuleNumber()]);
		}
		Logger.recordOutput("Swerve/Display/Target Swerve Module States",
				moduleStates);
	}

	@Override
	public boolean isConnected() { return gyro.isConnected(); }

	public void navXDisconnectProtocol() {
		if (gyro.isConnected() && debounce == 0) {
			DriveConstants.fieldOriented = true;
			return;
		} else {
			debounce = -1;
			DriveConstants.fieldOriented = false;
		}
	}

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }

	public void registerSelfCheckHardware() {
		super.registerHardware("IMU", gyro);
		super.registerHardware("FrontLeftDrive",
				m_swerveModules.get(ModulePosition.FRONT_LEFT).driveMotor);
		super.registerHardware("FrontLeftTurn",
				m_swerveModules.get(ModulePosition.FRONT_LEFT).turningMotor);
		super.registerHardware("FrontRightDrive",
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).driveMotor);
		super.registerHardware("FrontRightTurn",
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).turningMotor);
		super.registerHardware("BackLeftDrive",
				m_swerveModules.get(ModulePosition.BACK_LEFT).driveMotor);
		super.registerHardware("BackLeftTurn",
				m_swerveModules.get(ModulePosition.BACK_LEFT).turningMotor);
		super.registerHardware("BackRightDrive",
				m_swerveModules.get(ModulePosition.BACK_RIGHT).driveMotor);
		super.registerHardware("BackRightTurn",
				m_swerveModules.get(ModulePosition.BACK_RIGHT).turningMotor);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
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
				.sequence(run(() -> setChassisSpeeds(new ChassisSpeeds(1.5, 0, 0)))
						.withTimeout(1.0), runOnce(() -> {
							for (DriveConstants.TrainConstants.ModulePosition position : DriveConstants.TrainConstants.ModulePosition
									.values()) {
								// Retrieve the corresponding REVSwerveModule from the hashmap
								SwerveModuleState module = m_swerveModules.get(position)
										.getState();
								// Get the name of the current ModulePosition
								String name = position.name();
								if (Math.abs(module.speedMetersPerSecond) < 1.3
										|| Math.abs(module.speedMetersPerSecond) > 1.7) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow for "
													+ name + module.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = module.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 0) >= 10
										&& Math.abs(Math.abs(angle) - 180) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ module.angle.getDegrees(),
											false, true);
								}
							}
							System.out.println("COMPLETED X 1.5");
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1.5, 0)))
								.withTimeout(1.0),
						runOnce(() -> {
							System.out.println("STARTING Y CHECK...");
							for (DriveConstants.TrainConstants.ModulePosition position : DriveConstants.TrainConstants.ModulePosition
									.values()) {
								// Retrieve the corresponding REVSwerveModule from the hashmap
								SwerveModuleState module = m_swerveModules.get(position)
										.getState();
								// Get the name of the current ModulePosition
								String name = position.name();
								if (Math.abs(module.speedMetersPerSecond) < 1.3
										|| Math.abs(module.speedMetersPerSecond) > 1.7) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow for "
													+ name + module.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = module.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 90) >= 10
										&& Math.abs(Math.abs(angle) - 270) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ module.angle.getDegrees(),
											false, true);
								}
							}
						}),
						run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
								.withTimeout(2.0),
						run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
								.withTimeout(2.0))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp",
				m_swerveModules.get(ModulePosition.FRONT_LEFT).driveMotor
						.getMotorTemperature());
		tempMap.put("FLTurnTemp",
				m_swerveModules.get(ModulePosition.FRONT_LEFT).turningMotor
						.getMotorTemperature());
		tempMap.put("FRDriveTemp",
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).driveMotor
						.getMotorTemperature());
		tempMap.put("FRTurnTemp",
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).turningMotor
						.getMotorTemperature());
		tempMap.put("BLDriveTemp",
				m_swerveModules.get(ModulePosition.BACK_LEFT).driveMotor
						.getMotorTemperature());
		tempMap.put("BLTurnTemp",
				m_swerveModules.get(ModulePosition.BACK_LEFT).turningMotor
						.getMotorTemperature());
		tempMap.put("BRDriveTemp",
				m_swerveModules.get(ModulePosition.BACK_RIGHT).driveMotor
						.getMotorTemperature());
		tempMap.put("BRTurnTemp",
				m_swerveModules.get(ModulePosition.BACK_RIGHT).turningMotor
						.getMotorTemperature());
		return tempMap;
	}

	private boolean collisionDetected() {
		double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			return true;
		}
		return false;
	}

	@Override
	public boolean isCollisionDetected() { return collisionDetected; }

	@Override
	public double getCurrent() { return 0; }
}
