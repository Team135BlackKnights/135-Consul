package frc.robot.utils.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Compatibility helpers for the 2027 crap. */
public final class ChassisVelocityUtil {
  private ChassisVelocityUtil() {}

  public static ChassisVelocities fromFieldRelative(
      double vx, double vy, double omega, Rotation2d robotAngle) {
    return new ChassisVelocities(vx, vy, omega).toRobotRelative(robotAngle);
  }

  public static ChassisVelocities fromFieldRelative(
      ChassisVelocities fieldRelative, Rotation2d robotAngle) {
    return fieldRelative.toRobotRelative(robotAngle);
  }

  public static ChassisVelocities fromRobotRelative(
      ChassisVelocities robotRelative, Rotation2d robotAngle) {
    return robotRelative.toFieldRelative(robotAngle);
  }
}
