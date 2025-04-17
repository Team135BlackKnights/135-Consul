package frc.robot.commands.servos;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.servos.ServoS;
import frc.robot.utils.servos.ServoConstantContainer.ServoNames;
public class ServoC extends Command{
	private ServoS servoS;
	public ServoC(ServoS servoS){
		this.servoS = servoS;
		addRequirements(this.servoS);
	}
	@Override
	public void initialize(){

	}
	@Override
	public void execute(){
		
		//servoS.setServoDegrees(-110, ServoNames.leftServo);
		//servoS.setServoDegrees(45, ServoNames.rightServo);
		//servoS.setServoPercent(1); //go to 90
 		SmartDashboard.putNumber("current pos LEFT", servoS.getServoDegrees(ServoNames.leftServo));
		SmartDashboard.putNumber("current pos RIGHT", servoS.getServoDegrees(ServoNames.rightServo));
	}
	@Override
	public void end(boolean interrupted){

	}
	@Override
	public boolean isFinished() { return false; }

}
