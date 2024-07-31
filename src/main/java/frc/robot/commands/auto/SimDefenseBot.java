package frc.robot.commands.auto;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class SimDefenseBot extends Command {
	private Pose2d botPose;

	public SimDefenseBot() {}

	@Override
	public void initialize() {
		// Create a sequence of movements with interpolation
		botPose = new Pose2d(15, 6.7, new Rotation2d());
		Command sequence = new SequentialCommandGroup(
				interpolatePose(0, 8.2, 7.4, 5));//, interpolatePose(5, 8.3, 5.785, 5));
		// Schedule the sequence to start immediately
		CommandScheduler.getInstance().schedule(sequence);
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) { botPose = new Pose2d(); }

	@Override
	public boolean isFinished() { return false; }

	private Command interpolatePose(double startTime, double x, double y,
			double duration) {
		return new SequentialCommandGroup(Commands.defer(() -> {
			final Pose2d startPose = botPose;
			final Pose2d endPose = new Pose2d(x, y, new Rotation2d());
			final Timer timer = new Timer();
			timer.start();
			return Commands.run(() -> {
				botPose = startPose.interpolate(endPose, timer.get() / duration);
				//RobotContainer.opposingBotPose = botPose;
				Logger.recordOutput("BOTSIM", botPose);
			}).until(() -> timer.hasElapsed(duration));
		}, Set.of()));
	}
}