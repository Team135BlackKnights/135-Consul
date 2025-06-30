package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import frc.robot.utils.CompetitionFieldUtils.CompField;

/**
 * displays a game piece on field to Advantage Scope since game pieces MUST be
 * displayed as 3d objects in Advantage Scope we have to convert the 2d pose of
 * the game piece to a 3d pose
 */
public interface GamePieceOnFieldDisplay
		extends CompField.Object2dOnFieldDisplay {
	@Override
	default Pose3d getPose3d() {
		final Pose2d pose2d = getObjectOnFieldPose2d();
		final Translation3d translation3d = new Translation3d(pose2d.getX(),
				pose2d.getY(), getGamePieceHeight() / 2);
		return new Pose3d(translation3d, new Rotation3d(pose2d.getRotation()));
	}

	/**
	 * @return the height of the game piece when standing from ground, in meters
	 */
	double getGamePieceHeight();

	/**
	 * Checks if the game piece is in the score zone.
	 * 
	 * @return {@code true} if the game piece is in the score zone along with a string if needed for variable scoring, {@code false/Not In Score Zone} otherwise.
	 * 
	 */
	Pair<Boolean,String> isInScoreZone();

	/**
	 * the game piece's value AT THE CURRENT TIME IN THE MATCH for score.
	 */
	int getScoreValue(String scoreType);
	boolean shouldDeleteAndRemoveFromSimulation(String scoreType);
	/**
 * Returns the type of game piece that is currently scored (i.e., removed from play) in the simulation.
 *
 * <p><strong>DO NOT call {@code getPose3d()} inside this method — it will crash.</strong></p>
 *
 * <p><strong>Implementation Note:</strong> If scoring causes the game piece to disappear 
 * (e.g., a ball being shot or a repeatable scoring event at the same location), simply return {@code null}.
 * 
 * <p>As an example, in the 2025 game <em>Reefscape</em>, coral is considered a static object 
 * and would be placed on the reef with physics disabled in its constructor.
 
  @param scoreType the type of scoring event, e.g., "Coral", "AlgaeBall", etc. This is used to determine the type of game piece that was scored or transitioned.
 * @param oldPose the pose of the game piece just before it was scored or transitioned, in meters. Use instead of {@code getPose3d()} to avoid crashes.
 * @param magnitude the linear momentum of the game piece at the moment of scoring/transitioning, in meters per second
 * @param momentumAngle the angle of momentum at the moment of scoring/transitioning, in radians
 * @return the type of game piece that is currently scored, or {@code null} if no game piece is scored
 */
	GamePieceInSimulation scoredGamePieceType(String scoreType, Pose3d oldPose, double magnitude, double momentumAngle);

}
