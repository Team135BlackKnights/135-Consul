package frc.robot.utils.maths;
import java.util.function.DoubleSupplier;
/**
 * Improvised Kalman Filter class designed to take one variable. Needs a double supplier to output the actual function
 */
public class KalmanFilterImprov {
    private final DoubleSupplier systemSupplier;
    private final DoubleSupplier sensorSupplier;
    private final double systemStdDev;
    private final double zScoreCutoff;
    /**
     * Constructor for the Kalman Filter
     * @param systemSupplier the supplier for the system value (predictions done with kS kV kA)
     * @param sensorSupplier the supplier for the sensor value (output of the sensor)
     * @param systemStdDev the standard deviation of the system (this can be tuned)
     * @param zScoreCutoff the z-score cutoff for the sensor value (this is configurable/should be tuned)
     */
    public KalmanFilterImprov(DoubleSupplier systemSupplier, DoubleSupplier sensorSupplier, double systemStdDev, double zScoreCutoff){
        this.systemSupplier = systemSupplier;
        this.sensorSupplier = sensorSupplier;
        this.systemStdDev = systemStdDev;
        this.zScoreCutoff = zScoreCutoff;
    }
    /**
     * Filters the sensor value based on the system value. If the z-score cutoff is exceeded, the sensor value is used. If within z-score cutoff, the system value is used while factoring in std. devs
     * @return the filtered value
     */
    public double filter(){
        double systemValue = systemSupplier.getAsDouble();
        double sensorValue = sensorSupplier.getAsDouble();
        double kalmanGain = 1;
        double kalmanValue = systemValue;
        double zScore = Math.abs(sensorValue - systemValue) / systemStdDev;
        if(zScore < zScoreCutoff){
            kalmanGain = 1 - zScore / zScoreCutoff;
            kalmanValue = kalmanGain * sensorValue + (1 - kalmanGain) * systemValue;
        }
        return kalmanValue;
    }
    
}
