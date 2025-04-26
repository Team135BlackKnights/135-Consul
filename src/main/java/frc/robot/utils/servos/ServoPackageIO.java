package frc.robot.utils.servos;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;
public class ServoPackageIO implements ServoIO {
    private final ServoPackage servoPackage;
    private final SimServoMode servoMode;
    public ServoPackageIO(ServoPackage servoPackage) {
        this.servoPackage = servoPackage;
        this.servoMode = servoPackage.getSimServoMode();
    }
    @Override
    public void updateInputs(ServoIOInputs inputs) {
        if (servoMode == SimServoMode.CONTINUOUS) {
            inputs.PWMValue = servoPackage.getServoSpeedPercent();
            inputs.positionPercent = servoPackage.getServoVelocityDegreesPerSec()/servoPackage.getServoMaxVelocityDegreesPerSec();
            inputs.velocityDegreesPerSecond = servoPackage.getServoVelocityDegreesPerSec();
        } else {
            inputs.PWMValue = servoPackage.getServoPercent();
            inputs.positionDegrees = servoPackage.getServoPercent();
            inputs.velocityDegreesPerSecond = servoPackage.getServoDegrees();
        }
        servoPackage.updateServoSim();
    }
    public void setDegrees(double degrees) {
        if (servoMode == SimServoMode.INRANGE) {
            servoPackage.setServoDegrees(degrees);
        }      
    }
    public  void setPercent(double percent) {
        if (servoMode == SimServoMode.INRANGE) {
        servoPackage.setServoPercent(percent);
        }
    }
    public void setVelocity(double percent) {
        if (servoMode == SimServoMode.CONTINUOUS) {
            servoPackage.setServoDegreesPerSec(percent*servoPackage.getServoMaxVelocityDegreesPerSec());
        }
    }
    public void setVelocityDegrees(double degrees) {
        if (servoMode == SimServoMode.CONTINUOUS) {
            servoPackage.setServoDegreesPerSec(degrees);
        }
    }

}
