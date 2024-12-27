package frc.robot.commands.state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.Flywheel.FlywheelS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelC extends Command {
	private final FlywheelS flywheelS;
	private double flywheelSpeed;

	public FlywheelC(FlywheelS flywheelS) {
		this.flywheelS = flywheelS;
		addRequirements(flywheelS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.goto4000Button
				.whileTrue(new InstantCommand(() -> {
					flywheelSpeed = 4000;
					flywheelS.setRPM(flywheelSpeed);
				})).onFalse(new InstantCommand(() -> {
					flywheelSpeed =0;
					flywheelS.setRPM(flywheelSpeed);
				}));
	}

	@Override
	public void execute() {
		if (flywheelSpeed != 4000) {
			if (RobotContainer.manipController
					.getLeftTriggerAxis() > StateSpaceConstants.Controls.kDeadband) {
				flywheelSpeed = RobotContainer.manipController.getLeftTriggerAxis()
						* StateSpaceConstants.Flywheel.maxRPM;
			} else {
				flywheelSpeed = 0;
			}
			flywheelS.setRPM(flywheelSpeed);
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}