package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.Elevator.ElevatorS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorC extends Command {
	private final ElevatorS elevatorS;
	double elevatorPos =0;
	public ElevatorC(ElevatorS elevatorS) {
		this.elevatorS = elevatorS;
		addRequirements(elevatorS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.go0ftButton
						.onTrue(new InstantCommand(() -> {
							elevatorPos = Units.feetToMeters(0);
							elevatorS.setState(elevatorS.createState(elevatorPos));
						}));
		StateSpaceConstants.Controls.go2ftButton
						.onTrue(new InstantCommand(() -> {
							elevatorPos = Units.feetToMeters(2);
							elevatorS.setState(elevatorS.createState(elevatorPos));
						}));
	}

	@Override
	public void execute() {
		double elevatorSpeed = 0, desSpeed = -RobotContainer.manipController.getLeftY();
		elevatorPos = elevatorS.getSetpoint();
		if (Math.abs(desSpeed) > StateSpaceConstants.Controls.kArmDeadband) {
			if (elevatorPos >= StateSpaceConstants.Elevator.maxPosition && desSpeed > 0) {
				elevatorPos = StateSpaceConstants.Elevator.maxPosition;
			} else if (elevatorPos <= StateSpaceConstants.Elevator.startingPosition
					&& desSpeed < 0) {
				elevatorPos = StateSpaceConstants.Elevator.startingPosition;
			} else {
				elevatorSpeed = desSpeed * StateSpaceConstants.Elevator.maxAcceleration
						* StateSpaceConstants.Controls.elevatorMoveSpeed;
				elevatorPos += (elevatorSpeed * .02); //add to our current position 20 MS of that accel
			}
		}
		if (elevatorSpeed == 0) {
			elevatorS.setState(elevatorS.createState(elevatorPos));
		} else {
			elevatorS.setState(elevatorS.createState(elevatorPos, elevatorSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}