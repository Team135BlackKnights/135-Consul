package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants.GamePieceTag;
import frc.robot.utils.maths.GeometryConvertor;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.MassType;

/**
 * simulates the behavior of gamepiece on field. game pieces HAVE collision
 * spaces. they can also be "grabbed" by an Intake Simulation the game piece
 * will also be displayed on advantage scope (once registered in
 * CompetitionFieldSimulation)
 */
public abstract class GamePieceInSimulation extends Body
		implements GamePieceOnFieldDisplay {

	public final GamePieceTag tag;
	public GamePieceInSimulation(Translation2d initialPosition, Convex shape, GamePieceTag tag) {
		this(initialPosition, shape, FieldConstants.DEFAULT_MASS,tag);
	}

	public GamePieceInSimulation(Translation2d initialPosition, Convex shape,
			double mass, GamePieceTag tag) {
		super();
		BodyFixture bodyFixture = super.addFixture(shape);
		bodyFixture.setFriction(FieldConstants.EDGE_COEFFICIENT_OF_FRICTION);
		bodyFixture.setRestitution(FieldConstants.EDGE_COEFFICIENT_OF_RESTITUTION);
		bodyFixture.setDensity(mass / shape.getArea());
		this.tag = tag;
		super.setMass(MassType.NORMAL);
		super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));
		super.setLinearDamping(FieldConstants.LINEAR_DAMPING);
		super.setAngularDamping(FieldConstants.ANGULAR_DAMPING);
		super.setBullet(true);
	}
	public GamePieceTag getTag() {
		return tag;
	}
	@Override
	public Pose2d getObjectOnFieldPose2d() {
		return GeometryConvertor.toWpilibPose2d(super.getTransform());
	}
}
