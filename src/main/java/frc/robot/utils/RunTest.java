package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RunTest extends Command {
	private final SysIdRoutine.Direction direction;
	private final boolean isQuasiastic;
	private Command test;

	public RunTest(SysIdRoutine.Direction direction, boolean isQuasiastic) {
		this.direction = direction;
		this.isQuasiastic = isQuasiastic;
	}

	@Override
	public void initialize() {
		switch (Robot.runningTest) {
		case swerveDrive:
			if (isQuasiastic) {
				test = RobotContainer.drivetrainS.sysIdQuasistaticDrive(direction);
			} else {
				test = RobotContainer.drivetrainS.sysIdDynamicDrive(direction);
			}
			break;
		default:
			System.err.println(Robot.runningTest + " is not a valid test");
			break;
		}
		System.out.println("Running Test" + direction.toString());
		test.schedule();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) { test.cancel(); }

	@Override
	public void cancel() { test.cancel(); }

	@Override
	public boolean isFinished() { return test.isFinished(); }
}
