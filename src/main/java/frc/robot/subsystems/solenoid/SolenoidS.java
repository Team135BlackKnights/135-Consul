package frc.robot.subsystems.solenoid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.solenoid.SolenoidConstants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SolenoidS extends SubsystemBase { //nothing to check
	public static Solenoid singleSolenoid;
	public static DoubleSolenoid doubleSolenoid;
	public static double oscillationTime = .1;
	public static boolean pulseDouble,pulseSingle;
	private double lastSingle =0,lastDouble =0;

	/**
	 * Creates a new subsystem to handle solenoid sims. Holds example for a
	 * single-action and a double-action solenoid. Also works in sim
	 */
	public SolenoidS() {
		singleSolenoid = new Solenoid(PneumaticsModuleType.REVPH,
				SolenoidConstants.singleActingSolenoidChannel);
		doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
				SolenoidConstants.doubleActingSolenoidChannelForward,
				SolenoidConstants.doubleActingSolenoidChannelBackward);
		singleSolenoid.setPulseDuration(oscillationTime);
	}
	/**
	 * Sets the state of the single solenoid
	 * @param on whether it is on or off (off is false)
	 */
	public static void setSingleSolenoid(boolean on){
		singleSolenoid.set(on);
	}
	public static void setSingleSolenoidOscillation(){
		pulseSingle = true;
	}
	/**
	 * @return the state of the single solenoid as a value (literally read the line above goober)
	 */
	public static boolean getSingleSolenoid(){
		return singleSolenoid.get();
	}
	/**
	 * Set the state of the double solenoid
	 * @param state the desired state of the double solenoid (Value.kForward, Value.kReverse, Value.kOff)
	 */
	public static void setDoubleSolenoid(DoubleSolenoid.Value state){
		doubleSolenoid.set(state);
	}
	public static void setDoubleSolenoidOscillation(){
		pulseDouble = true;
	}
	/**
	 * 
	 * @return the value of the solenoid (Either Value.kFoward, Value.kReverse, Value.kOff)
	 */
	public static DoubleSolenoid.Value getDoubleSolenoidState(){
		return doubleSolenoid.get();
	}
	public void goToValue(double heightInTime,boolean isDouble){
		new InstantCommand(() -> {
			Timer timer = new Timer();
			while (timer.get() < heightInTime){
				if (isDouble){
					doubleSolenoid.set(Value.kForward);
				}else{
					singleSolenoid.set(true);
				}
			}
			if(isDouble){
				SolenoidS.pulseDouble = true;
			}else{
				SolenoidS.pulseSingle = true;
			}
		}, this).schedule();
	}
	public void stopSinglePulse(){
		pulseSingle = false;
	}
	public void stopDoublePulse(){
		pulseDouble = false;
	}
	@Override
	public void periodic(){
		double currentTime = Timer.getFPGATimestamp();
		if (currentTime - lastSingle > oscillationTime && pulseSingle){
			lastSingle = currentTime;
			singleSolenoid.startPulse();
		}
		if (currentTime - lastDouble > oscillationTime && pulseDouble){
			lastDouble = currentTime;
			doubleSolenoid.toggle();
		}
	}
}
