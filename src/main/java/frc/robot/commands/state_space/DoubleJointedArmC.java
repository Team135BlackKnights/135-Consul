package frc.robot.commands.state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.state_space.DoubleJointedArm.DoubleJointedArmS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class DoubleJointedArmC extends Command {
	private final DoubleJointedArmS armS;

	public DoubleJointedArmC(DoubleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.gotoUpRight
				.onTrue(new InstantCommand(() -> armS.setDoubleJointedArm(StateSpaceConstants.DoubleJointedArm.macroTopRight)));
		StateSpaceConstants.Controls.gotoUpLeft
						.onTrue(new InstantCommand(() -> armS.setDoubleJointedArm(StateSpaceConstants.DoubleJointedArm.macroTopLeft)));
	}

	@Override
	public void execute() {
		
	}
}
