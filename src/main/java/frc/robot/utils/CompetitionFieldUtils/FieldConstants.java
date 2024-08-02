package frc.robot.utils.CompetitionFieldUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the field MUST CREATE A NEW CLASS FOR EACH NEW GAME, @see
 * Crescendo2024FieldSimulation & @see Crescendo2024FieldObjects for example
 */
public class FieldConstants {
	public static final double FIELD_WIDTH = 16.54;
	public static final double FIELD_HEIGHT = 8.21;
	//id 1 is topmost leftmost. goes in order down, right.
	//Speaker translations
	public static final Pose2d START_POSE = new Pose2d(1.5, 1.5,
			new Rotation2d());
	public static final Translation3d BLUE_SPEAKER = new Translation3d(0.225,
			5.55, 2.1);
	public static final Translation3d RED_SPEAKER = new Translation3d(16.317,
			5.55, 2.1); //in meters!
	public static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
			new Translation2d(2.9, 4.1), new Translation2d(2.9, 5.55),
			new Translation2d(2.9, 7), new Translation2d(8.27, 0.75),
			new Translation2d(8.27, 2.43), new Translation2d(8.27, 4.1),
			new Translation2d(8.27, 5.78), new Translation2d(8.27, 7.46),
			new Translation2d(13.64, 4.1), new Translation2d(13.64, 5.55),
			new Translation2d(13.64, 7),
	};
	/* https://www.andymark.com/products/frc-2024-am-4999 */
	public static final double NOTE_HEIGHT = Units.inchesToMeters(2),
			NOTE_DIAMETER = Units.inchesToMeters(14);

	public enum GamePieceTag {
		ON_GROUND, IN_ROBOT, IN_AIR
	}
}
