package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.FieldConstants;
import frc.robot.utils.maths.GeometryConvertor;
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
	public GamePieceInSimulation(Translation2d initialPosition, Convex shape, boolean isAlgaeBall) {
		this(initialPosition, shape, isAlgaeBall ? FieldConstants.AlgaeBall.DEFAULT_MASS_KG : FieldConstants.ReefscapeCoral.DEFAULT_MASS_KG,0,0);
	}
	public GamePieceInSimulation(Translation2d initialPosition, Convex shape, double momentumAngle, double momentumMagnitude,boolean isAlgaeBall) {
		this(initialPosition, shape, isAlgaeBall ? FieldConstants.AlgaeBall.DEFAULT_MASS_KG : FieldConstants.ReefscapeCoral.DEFAULT_MASS_KG, momentumAngle, momentumMagnitude);
	}

	public GamePieceInSimulation(Translation2d initialPosition, Convex shape,
			double mass , double momentumAngle, double momentumMagnitude) {
		super();
		BodyFixture bodyFixture = super.addFixture(shape);
		if (mass == FieldConstants.AlgaeBall.DEFAULT_MASS_KG){
			bodyFixture.setFriction(FieldConstants.AlgaeBall.EDGE_COEFFICIENT_OF_FRICTION);
			bodyFixture.setRestitution(FieldConstants.AlgaeBall.EDGE_COEFFICIENT_OF_RESTITUTION);
			bodyFixture.setDensity(mass / shape.getArea());
			super.setLinearDamping(FieldConstants.AlgaeBall.LINEAR_DAMPING);
			super.setAngularDamping(FieldConstants.AlgaeBall.ANGULAR_DAMPING);
		}else{
			bodyFixture.setFriction(FieldConstants.ReefscapeCoral.EDGE_COEFFICIENT_OF_FRICTION);
			bodyFixture.setRestitution(FieldConstants.ReefscapeCoral.EDGE_COEFFICIENT_OF_RESTITUTION);
			bodyFixture.setDensity(mass / shape.getArea());
			super.setLinearDamping(FieldConstants.ReefscapeCoral.LINEAR_DAMPING);
			super.setAngularDamping(FieldConstants.ReefscapeCoral.ANGULAR_DAMPING);
		}
		
		super.setMass(MassType.NORMAL);
		super.translate(GeometryConvertor.toDyn4jVector2(initialPosition));
		super.rotateAboutCenter(momentumAngle);
		super.setBullet(true);
		super.setLinearVelocity(Vector2.create(momentumMagnitude, momentumAngle));
	}
	@Override
	public Pose2d getObjectOnFieldPose2d() {
		return GeometryConvertor.toWpilibPose2d(super.getTransform());
	}
}
