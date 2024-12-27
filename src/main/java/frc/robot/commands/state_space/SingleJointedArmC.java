package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class SingleJointedArmC extends Command {
	private final SingleJointedArmS armS;
	double armPos = 0;
	public SingleJointedArmC(SingleJointedArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.go45Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(45);
							armS.setState(armS.createState(armPos));
						}));
		StateSpaceConstants.Controls.go0Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(0);
							armS.setState(armS.createState(armPos));
						}));
	}

	@Override
	public void execute() {
		double armSpeed = 0, desSpeed = -RobotContainer.manipController.getRightY();
		armPos = armS.getSetpoint();
		if (Math.abs(desSpeed) > StateSpaceConstants.Controls.kArmDeadband) {
			if (armPos >= StateSpaceConstants.SingleJointedArm.maxPosition && desSpeed > 0) {
				armPos = StateSpaceConstants.SingleJointedArm.maxPosition;
			} else if (armPos <= StateSpaceConstants.SingleJointedArm.startingPosition
					&& desSpeed < 0) {
				armPos = StateSpaceConstants.SingleJointedArm.startingPosition;
			} else {
				armSpeed = desSpeed * StateSpaceConstants.SingleJointedArm.maxAcceleration
						* StateSpaceConstants.Controls.armMoveSpeed;
				armPos += (armSpeed * .02); //add to our current position 20 MS of that accel
			}
		}
		if (armSpeed == 0) {
			armS.setState(armS.createState(armPos));
		} else {
			armS.setState(armS.createState(armPos, armSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}