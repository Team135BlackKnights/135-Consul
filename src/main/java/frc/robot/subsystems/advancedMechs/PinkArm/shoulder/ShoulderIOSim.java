package frc.robot.subsystems.advancedMechs.PinkArm.shoulder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.MapleMotorSim;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimMotorConfigs;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedBattery;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedMotorController;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Shoulder;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.List;

/** Simulation implementation for the Pink Arm shoulder assembly. */
public class ShoulderIOSim implements ShoulderIO {
    private enum ControlMode {
        DUTY_CYCLE,
        POSITION
    }

    private static final int MOTOR_COUNT = 4;
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double AMBIENT_TEMP_C = 30.0;
    private static final double MAX_TEMP_C = 95.0;

    private final MapleMotorSim motorSim;
    private final SimulatedMotorController.GenericMotorController motorController;
    private final PIDController positionController;
    private SimpleMotorFeedforward feedforward;
    private TrapezoidProfile.Constraints profileConstraints;

    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State setpointState;
    private ControlMode controlMode = ControlMode.POSITION;
    private double dutyCycleCommand = 0.0;
    private double sensorOffsetRadians = 0.0;
    private double previousVelocityRadPerSec = 0.0;
    private double estimatedMotorTempC = AMBIENT_TEMP_C;
    private final double forwardLimitRad;
    private final double reverseLimitRad;
    private int statorCurrentLimitPerMotor = (int) Shoulder.statorCurrentLimit;

    public ShoulderIOSim() {
    forwardLimitRad = Shoulder.maxPosition;
    reverseLimitRad = -Units.degreesToRadians(0);

    SimMotorConfigs configs = new SimMotorConfigs(
        DCMotor.getKrakenX60Foc(MOTOR_COUNT),
        Shoulder.shoulderGearing,
        KilogramSquareMeters.of(Shoulder.shoulderMOI),
        Volts.of(Math.max(Shoulder.kS.get(), 0.01))).withHardLimits(Radians.of(Double.POSITIVE_INFINITY), Radians.of(reverseLimitRad));
        motorSim = new MapleMotorSim(configs);
        motorController = motorSim.useSimpleDCMotorController().withCurrentLimit(Amps.of(statorCurrentLimitPerMotor * MOTOR_COUNT)).withSoftwareLimits(Radians.of(Double.POSITIVE_INFINITY), Radians.of(reverseLimitRad));

        positionController = new PIDController(Shoulder.kP.get(), Shoulder.kI.get(), Shoulder.kD.get());
        feedforward = new SimpleMotorFeedforward(Shoulder.kS.get(), Shoulder.kV.get());
        profileConstraints = createConstraints();

        double initialInternalPosition = motorSim.getAngularPosition().in(Radians);
        Rotation2d startingAngle = Rotation2d.fromDegrees(Shoulder.startingPosition);
        sensorOffsetRadians = startingAngle.getRadians() - initialInternalPosition;

        goalState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        refreshTuningsIfNeeded();
        runControlStep();

        double mechanismPositionRad = clampToLimits(motorSim.getAngularPosition().in(Radians));  
        double mechanismVelocityRadPerSec = motorSim.getVelocity().in(RadiansPerSecond);
        double angularAcceleration = (mechanismVelocityRadPerSec - previousVelocityRadPerSec) / LOOP_PERIOD_SEC;
        previousVelocityRadPerSec = mechanismVelocityRadPerSec;

        double totalStatorCurrent = Math.abs(motorSim.getStatorCurrent().in(Amps));
        updateThermalModel(totalStatorCurrent);

        double perMotorStatorCurrent = totalStatorCurrent / MOTOR_COUNT;
        double perMotorSupplyCurrent = motorSim.getSupplyCurrent().in(Amps) / MOTOR_COUNT;

        inputs.shoulderAngle = Rotation2d.fromRadians(mechanismPositionRad + sensorOffsetRadians);
        inputs.shoulderAppliedVolts = motorSim.getAppliedVoltage().in(Volts);
        inputs.shoulderSupplyCurrentAmps = perMotorSupplyCurrent;
        inputs.shoulderStatorCurrentAmps = perMotorStatorCurrent;
        inputs.shoulderAngularVelocityRadPerSec = mechanismVelocityRadPerSec;
        inputs.shoulderAngularAccelerationRadPerSecSquared = angularAcceleration;

        inputs.shoulderFRMotorTemp = estimatedMotorTempC;
        inputs.shoulderFLMotorTemp = estimatedMotorTempC;
        inputs.shoulderBRMotorTemp = estimatedMotorTempC;
        inputs.shoulderBLMotorTemp = estimatedMotorTempC;
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        double desiredActualRadians = clampToLimits(target.getRadians());
        double internalTarget = desiredActualRadians - sensorOffsetRadians;
        goalState = new TrapezoidProfile.State(internalTarget, 0.0);
        setpointState = new TrapezoidProfile.State(
                clampToLimits(motorSim.getAngularPosition().in(Radians)),
                motorSim.getVelocity().in(RadiansPerSecond));
        controlMode = ControlMode.POSITION;
        dutyCycleCommand = 0.0;
    }

    @Override
    public void resetShoulderAngle(Rotation2d angle) {
        double actualPosition = motorSim.getAngularPosition().in(Radians);
        actualPosition = clampToLimits(actualPosition);
        sensorOffsetRadians = angle.getRadians() - actualPosition;
        setpointState = new TrapezoidProfile.State(actualPosition, motorSim.getVelocity().in(RadiansPerSecond));
        goalState = new TrapezoidProfile.State(actualPosition, 0.0);
        positionController.reset();
        controlMode = ControlMode.POSITION;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        controlMode = ControlMode.DUTY_CYCLE;
        dutyCycleCommand = MathUtil.clamp(dutyCycle, -1.0, 1.0);
    }

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setCurrentLimit(int current) {
        statorCurrentLimitPerMotor = Math.max(1, current);
        motorController.withCurrentLimit(Amps.of(statorCurrentLimitPerMotor * MOTOR_COUNT));
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return List.of();
    }

    private void runControlStep() {
        switch (controlMode) {
            case POSITION -> runPositionControl();
            case DUTY_CYCLE -> runDutyCycleControl();
        }

        motorSim.update(Seconds.of(LOOP_PERIOD_SEC));
    }

    private void runPositionControl() {
        TrapezoidProfile profile = new TrapezoidProfile(profileConstraints);
        setpointState = profile.calculate(LOOP_PERIOD_SEC, setpointState, goalState);
        positionController.setSetpoint(setpointState.position);

        double measured = clampToLimits(motorSim.getAngularPosition().in(Radians));
        
        double pidVolts = positionController.calculate(measured);
        double ffVolts = feedforward.calculate(setpointState.velocity);
        double gravityVolts = Shoulder.kG.get() * Math.cos(
                (measured + sensorOffsetRadians) - Units.degreesToRadians(Shoulder.shoulderMOIDegreesFromZero));

        System.out.println("measured: " + Units.radiansToDegrees(measured) + " pidVolts: " + pidVolts + " ffVolts: " + ffVolts + " gravityVolts: " + gravityVolts);
        requestVoltage(pidVolts + ffVolts + gravityVolts);
    }

    private void runDutyCycleControl() {
        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        requestVoltage(batteryVoltage * dutyCycleCommand);
    }

    private void requestVoltage(double volts) {
        double applied = Shoulder.inverted ? -volts : volts;
        motorController.requestVoltage(Volts.of(applied));
    }

    private void refreshTuningsIfNeeded() {
        LoggableTunedNumber.ifChanged(
                hashCode(),
                () -> {
                    positionController.setPID(Shoulder.kP.get(), Shoulder.kI.get(), Shoulder.kD.get());
                    feedforward = new SimpleMotorFeedforward(Shoulder.kS.get(), Shoulder.kV.get());
                    profileConstraints = createConstraints();
                },
                Shoulder.kP,
                Shoulder.kI,
                Shoulder.kD,
                Shoulder.kS,
                Shoulder.kV,
                Shoulder.kG,
                Shoulder.maxSpeed,
                Shoulder.maxAcceleration);
    }

    private TrapezoidProfile.Constraints createConstraints() {
        double maxVelRadPerSec = Units.degreesToRadians(Shoulder.maxSpeed.get());
        double maxAccelRadPerSecSq = Units.degreesToRadians(Shoulder.maxAcceleration.get());
        return new TrapezoidProfile.Constraints(maxVelRadPerSec, maxAccelRadPerSecSq);
    }

    private double clampToLimits(double radians) {
        return MathUtil.clamp(radians, reverseLimitRad, forwardLimitRad);
    }

    private void updateThermalModel(double totalStatorCurrent) {
        double perMotorCurrent = totalStatorCurrent / MOTOR_COUNT;
        double normalized = MathUtil.clamp(perMotorCurrent / Math.max(1.0, statorCurrentLimitPerMotor), 0.0, 2.0);
        double heatingPerSec = 15.0 * normalized;
        double coolingPerSec = (estimatedMotorTempC - AMBIENT_TEMP_C) * 0.3;
        estimatedMotorTempC += (heatingPerSec - coolingPerSec) * LOOP_PERIOD_SEC;
        estimatedMotorTempC = MathUtil.clamp(estimatedMotorTempC, AMBIENT_TEMP_C, MAX_TEMP_C);
    }
}
