package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.maths.GeometryConvertor;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

/**
 * simulates the behavior of gamepiece on field. game pieces HAVE collision
 * spaces. they can also be "grabbed" by an Intake Simulation the game piece
 * will also be displayed on advantage scope (once registered in
 * CompetitionFieldSimulation)
 */
public abstract class GamePieceInSimulation extends Body
		implements GamePieceOnFieldDisplay {
	public double momentumAngle = 0;
	public double momentumMagnitude = 0;

	public GamePieceInSimulation(Translation2d initialPosition, Convex shape) {
		this(initialPosition, shape, FieldConstants.Fuel.DEFAULT_MASS_KG, 0, 0);
	}

	public GamePieceInSimulation(Translation2d initialPosition, Convex shape, double momentumAngle,
			double momentumMagnitude) {
		this(initialPosition, shape, FieldConstants.Fuel.DEFAULT_MASS_KG, momentumAngle, momentumMagnitude);
	}

	public GamePieceInSimulation(Translation2d initialPosition, Convex shape,
			double mass, double momentumAngle, double momentumMagnitude) {
		super();
		BodyFixture bodyFixture = super.addFixture(shape);
		bodyFixture.setFriction(FieldConstants.Fuel.EDGE_COEFFICIENT_OF_FRICTION);
		bodyFixture.setRestitution(FieldConstants.Fuel.EDGE_COEFFICIENT_OF_RESTITUTION);
		bodyFixture.setDensity(mass / shape.getArea());
		super.setLinearDamping(FieldConstants.Fuel.LINEAR_DAMPING);
		super.setAngularDamping(FieldConstants.Fuel.ANGULAR_DAMPING);

		super.setMass(MassType.NORMAL);
		super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));
		super.rotateAboutCenter(momentumAngle);
		super.setBullet(true);
		super.setLinearVelocity(Vector2.create(momentumMagnitude, momentumAngle));
		// zero out all the air stuff
		this.initialPosition = initialPosition;
		this.initialLaunchingVelocityMPS = new Translation2d(0, 0);
		this.initialHeight = 0;
		this.initialVerticalSpeedMPS = 0;
		this.gamePieceRotation = new Rotation3d(0, 0, 0);
		this.launchedTimer = new Timer();

	}

	@Override
	public Pose2d getObjectOnFieldPose2d() {
		return GeometryConvertor.toWpilibPose2d(super.getTransform());
	}

	@Override
	public Pose3d getPose3d() {
		if (launchedTimer.isRunning()) {
			return new Pose3d(getPositionAtTime(launchedTimer.get()), gamePieceRotation);
		}
		return GamePieceOnFieldDisplay.super.getPose3d();
	}

	// Air stuff
	public static final double GRAVITY = FieldConstants.COEFFICIENT_OF_GRAVITY;

	// Properties of the game piece projectile:
	protected final String gamePieceType = "Fuel";
	protected final Translation2d initialPosition;
	protected final Translation2d initialLaunchingVelocityMPS;
	protected final double initialHeight, initialVerticalSpeedMPS;
	protected final Rotation3d gamePieceRotation;
	protected final Timer launchedTimer;
	private Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackHitTarget = projectileTrajectory -> {
	};

	private Consumer<List<Pose3d>> projectileTrajectoryDisplayCallBackMiss = projectileTrajectory -> {
	};

	// Optional properties of the game piece, used if we want it to become a
	// GamePieceOnFieldSimulation upon touching ground:
	protected boolean becomesGamePieceOnGroundAfterTouchGround = false;

	// Optional properties of the game piece, used if we want it to have a target:
	private Translation3d tolerance = new Translation3d(0.2, 0.2, 0.2);
	private Supplier<Translation3d> targetPositionSupplier = () -> new Translation3d(0, 0, -100);
	private Runnable hitTargetCallBack = () -> {
	};
	private double heightAsTouchGround = 0.5;
	private double calculatedHitTargetTime = -1;
	@Override
    public String getType() {
        return this.gamePieceType;
    }
	public GamePieceInSimulation(
			Convex shape,
			double mass,
			Translation2d robotPosition,
			Translation2d shooterPositionOnRobot,
			ChassisSpeeds chassisSpeedsFieldRelative,
			Rotation2d shooterFacing,
			Distance initialHeight,
			LinearVelocity launchingSpeed,
			Angle shooterAngle) {
		this(
				shape,
				mass,
				robotPosition.plus(shooterPositionOnRobot.rotateBy(shooterFacing)),
				calculateInitialProjectileVelocityMPS(
						shooterPositionOnRobot,
						chassisSpeedsFieldRelative,
						shooterFacing,
						launchingSpeed.in(MetersPerSecond) * Math.cos(shooterAngle.in(Radians))),
				initialHeight.in(Meters),
				launchingSpeed.in(MetersPerSecond) * Math.sin(shooterAngle.in(Radians)),
				new Rotation3d(0, -shooterAngle.in(Radians), shooterFacing.getRadians()));
	}

	private static Translation2d calculateInitialProjectileVelocityMPS(
			Translation2d shooterPositionOnRobot,
			ChassisSpeeds chassisSpeeds,
			Rotation2d chassisFacing,
			double groundSpeedMPS) {
		final Translation2d chassisTranslationalVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond),
				shooterGroundVelocityDueToChassisRotation = shooterPositionOnRobot
						.rotateBy(chassisFacing)
						.rotateBy(Rotation2d.fromDegrees(90))
						.times(chassisSpeeds.omegaRadiansPerSecond),
				shooterGroundVelocity = chassisTranslationalVelocity.plus(shooterGroundVelocityDueToChassisRotation);

		return shooterGroundVelocity.plus(new Translation2d(groundSpeedMPS, chassisFacing));
	}

	public GamePieceInSimulation(Convex shape,
			double mass,
			Translation2d initialPosition,
			Translation2d initialLaunchingVelocityMPS,
			double initialHeight,
			double initialVerticalSpeedMPS,
			Rotation3d gamePieceRotation) {
		super();
		this.initialPosition = initialPosition;
		this.initialLaunchingVelocityMPS = initialLaunchingVelocityMPS;
		this.initialHeight = initialHeight;
		this.initialVerticalSpeedMPS = initialVerticalSpeedMPS;
		this.gamePieceRotation = gamePieceRotation;
		this.launchedTimer = new Timer();
		BodyFixture bodyFixture = super.addFixture(shape);
		bodyFixture.setFriction(FieldConstants.Fuel.EDGE_COEFFICIENT_OF_FRICTION);
		bodyFixture.setRestitution(FieldConstants.Fuel.EDGE_COEFFICIENT_OF_RESTITUTION);
		bodyFixture.setDensity(mass / shape.getArea());
		super.setLinearDamping(FieldConstants.Fuel.LINEAR_DAMPING);
		super.setAngularDamping(FieldConstants.Fuel.ANGULAR_DAMPING);

		super.setMass(MassType.NORMAL);
		super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));
		super.rotateAboutCenter(momentumAngle);
		super.setBullet(true);
		super.setLinearVelocity(Vector2.create(momentumMagnitude, momentumAngle));
	}

	public void launch() {
		final int maxIterations = 100;
		final double stepSeconds = 0.02;
		List<Pose3d> trajectoryPoints = new ArrayList<>();

		for (int i = 0; i < maxIterations; i++) {
			final double t = i * stepSeconds;
			final Translation3d currentPosition = getPositionAtTime(t);
			trajectoryPoints.add(new Pose3d(currentPosition, gamePieceRotation));

			if (currentPosition.getZ() < heightAsTouchGround && t * GRAVITY > initialVerticalSpeedMPS)
				break;
			if (isOutOfField(t))
				break;
			final Translation3d displacementToTarget = targetPositionSupplier.get().minus(currentPosition);
			if (Math.abs(displacementToTarget.getX()) < tolerance.getX()
					&& Math.abs(displacementToTarget.getY()) < tolerance.getY()
					&& Math.abs(displacementToTarget.getZ()) < tolerance.getZ()) {
				this.calculatedHitTargetTime = t;
				break;
			}
		}
		if (willHitTarget())
			projectileTrajectoryDisplayCallBackHitTarget.accept(trajectoryPoints);
		else
			projectileTrajectoryDisplayCallBackMiss.accept(trajectoryPoints);

		launchedTimer.reset();
		launchedTimer.start();
	}

	public boolean hasHitGround() {
		return getPositionAtTime(launchedTimer.get()).getZ() <= heightAsTouchGround
				&& launchedTimer.get() * GRAVITY > initialVerticalSpeedMPS;
	}

	public boolean hasGoneOutOfField() {
		return isOutOfField(launchedTimer.get());
	}

	private boolean isOutOfField(double time) {
		final Translation3d position = getPositionAtTime(time);
		final double EDGE_TOLERANCE = 2;
		return position.getX() < -EDGE_TOLERANCE
				|| position.getX() > FieldConstants.FIELD_WIDTH + EDGE_TOLERANCE
				|| position.getY() < -EDGE_TOLERANCE
				|| position.getY() > FieldConstants.FIELD_HEIGHT + EDGE_TOLERANCE;
	}

	public boolean willHitTarget() {
		return calculatedHitTargetTime != -1;
	}

	public boolean hasHitTarget() {
		return willHitTarget() && launchedTimer.get() >= calculatedHitTargetTime;
	}

	public GamePieceInSimulation cleanUp() {
		this.projectileTrajectoryDisplayCallBackHitTarget.accept(new ArrayList<>());
		this.projectileTrajectoryDisplayCallBackMiss.accept(new ArrayList<>());
		return this;
	}

	protected Translation3d getPositionAtTime(double t) {
		final double height = initialHeight + initialVerticalSpeedMPS * t - 1.0 / 2.0 * GRAVITY * t * t;

		final Translation2d current2dPosition = initialPosition.plus(initialLaunchingVelocityMPS.times(t));
		return new Translation3d(current2dPosition.getX(), current2dPosition.getY(), height);
	}

	@SuppressWarnings("unused")
	private Translation3d getVelocityMPSAtTime(double t) {
		final double verticalVelocityMPS = initialVerticalSpeedMPS - GRAVITY * t;

		return new Translation3d(
				initialLaunchingVelocityMPS.getX(), initialLaunchingVelocityMPS.getY(), verticalVelocityMPS);
	}

	public void triggerHitTargetCallBack() {
		hitTargetCallBack.run();
	}
public GamePieceInSimulation withTouchGroundHeight(double heightAsTouchGround) {
        this.heightAsTouchGround = heightAsTouchGround;
        return this;
    }
	public GamePieceInSimulation enableBecomesGamePieceOnFieldAfterTouchGround() {
        this.becomesGamePieceOnGroundAfterTouchGround = true;
        return this;
    }
	public GamePieceInSimulation disableBecomesGamePieceOnFieldAfterTouchGround() {
        this.becomesGamePieceOnGroundAfterTouchGround = false;
        return this;
    }
	public void setHitTargetCallBack(Runnable hitTargetCallBack) {
		this.hitTargetCallBack = hitTargetCallBack;
	}

	public boolean shouldBecomeGamePieceOnFieldAfterTouchGround() {
		return becomesGamePieceOnGroundAfterTouchGround;
	}

	public boolean isVisibleToSimObjectDetection() {
		return isGrounded();
	}
}
