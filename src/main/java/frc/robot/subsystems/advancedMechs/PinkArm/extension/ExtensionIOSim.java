package frc.robot.subsystems.advancedMechs.PinkArm.extension;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.MapleMotorSim;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimMotorConfigs;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedBattery;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedMotorController;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Extension;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.List;

/** Simulation-backed implementation of the Pink Arm extension mechanism. */
public class ExtensionIOSim implements ExtensionIO {
    private enum ControlMode {
        DUTY_CYCLE,
        POSITION
    }

    private static final int MOTOR_COUNT = 2;
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double AMBIENT_TEMP_C = 30.0;
    private static final double MAX_TEMP_C = 95.0;
    private static final double METERS_PER_RADIAN = Extension.extensionPositionCoefficient / (2.0 * Math.PI);
    private static final double RADIANS_PER_METER = 1.0 / METERS_PER_RADIAN;

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
    private int statorCurrentLimitPerMotor = (int) Extension.statorCurrentLimit;

    public ExtensionIOSim() {
        forwardLimitRad = Extension.maxPosition * RADIANS_PER_METER;
        reverseLimitRad = Units.inchesToMeters(-2.0) * RADIANS_PER_METER;

        SimMotorConfigs configs = new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(MOTOR_COUNT),
                Extension.extensionGearing,
                KilogramSquareMeters.of(Extension.extensionMOI),
                Volts.of(Math.max(Extension.kS.get(), 0.01)))
                .withHardLimits(Radians.of(Double.POSITIVE_INFINITY), Radians.of(reverseLimitRad));

        motorSim = new MapleMotorSim(configs);
        motorController = motorSim.useSimpleDCMotorController().withCurrentLimit(Amps.of(statorCurrentLimitPerMotor * MOTOR_COUNT)).withSoftwareLimits(Radians.of(Double.POSITIVE_INFINITY), Radians.of(reverseLimitRad));
        positionController = new PIDController(Extension.kP.get(), Extension.kI.get(), Extension.kD.get());
        feedforward = new SimpleMotorFeedforward(Extension.kS.get(), Extension.kV.get());
        profileConstraints = createConstraints();

        double initialInternalPosition = motorSim.getAngularPosition().in(Radians);
        double startingPositionRad = Extension.startingPosition * RADIANS_PER_METER;
        sensorOffsetRadians = startingPositionRad - initialInternalPosition;

        goalState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
    }

    @Override
    public void updateInputs(ExtensionIOInputs inputs) {
        refreshTuningsIfNeeded();
        runControlStep();

        double mechanismPositionRad = motorSim.getAngularPosition().in(Radians);
        double mechanismVelocityRadPerSec = motorSim.getVelocity().in(RadiansPerSecond);
        double angularAcceleration = (mechanismVelocityRadPerSec - previousVelocityRadPerSec) / LOOP_PERIOD_SEC;
        previousVelocityRadPerSec = mechanismVelocityRadPerSec;

        double totalStatorCurrent = Math.abs(motorSim.getStatorCurrent().in(Amps));
        updateThermalModel(totalStatorCurrent);

        double extensionMeters = radiansToMeters(mechanismPositionRad + sensorOffsetRadians);
        double velocityMetersPerSec = mechanismVelocityRadPerSec * METERS_PER_RADIAN;
        double accelerationMetersPerSecSq = angularAcceleration * METERS_PER_RADIAN;

        double perMotorSupplyCurrent = Math.abs(motorSim.getSupplyCurrent().in(Amps)) / MOTOR_COUNT;
        double perMotorStatorCurrent = totalStatorCurrent / MOTOR_COUNT;

        inputs.extensionPositionInMeters = extensionMeters;
        inputs.extensionAppliedVolts = motorSim.getAppliedVoltage().in(Volts);
        inputs.extensionSupplyCurrentAmps = perMotorSupplyCurrent;
        inputs.extensionStatorCurrentAmps = perMotorStatorCurrent;
        inputs.extensionVelocityMetersPerSec = velocityMetersPerSec;
        inputs.extensionAccelerationMetersPerSecSquared = accelerationMetersPerSecSq;
        inputs.extensionOneMotorTemp = estimatedMotorTempC;
        inputs.extensionTwoMotorTemp = estimatedMotorTempC;
    }

    @Override
    public void setTargetExtension(double positionInMeters) {
        double targetRad = clampRadians(positionInMeters * RADIANS_PER_METER);
        double internalTarget = targetRad - sensorOffsetRadians;
        goalState = new TrapezoidProfile.State(internalTarget, 0.0);
        setpointState = new TrapezoidProfile.State(
                motorSim.getAngularPosition().in(Radians),
                motorSim.getVelocity().in(RadiansPerSecond));
        controlMode = ControlMode.POSITION;
        dutyCycleCommand = 0.0;
    }

    @Override
    public void resetExtensionPosition(double positionInMeters) {
        double actualPosition = motorSim.getAngularPosition().in(Radians);
        sensorOffsetRadians = positionInMeters * RADIANS_PER_METER - actualPosition;
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

        double measured = motorSim.getAngularPosition().in(Radians);
        double pidVolts = positionController.calculate(measured);
        double ffVolts = feedforward.calculate(setpointState.velocity);
        requestVoltage(pidVolts + ffVolts);
    }

    private void runDutyCycleControl() {
        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        requestVoltage(dutyCycleCommand * batteryVoltage);
    }

    private void requestVoltage(double volts) {
        double applied = Extension.inverted ? -volts : volts;
        motorController.requestVoltage(Volts.of(applied));
    }

    private void refreshTuningsIfNeeded() {
        LoggableTunedNumber.ifChanged(
                hashCode(),
                () -> {
                    positionController.setPID(Extension.kP.get(), Extension.kI.get(), Extension.kD.get());
                    feedforward = new SimpleMotorFeedforward(Extension.kS.get(), Extension.kV.get());
                    profileConstraints = createConstraints();
                },
                Extension.kP,
                Extension.kI,
                Extension.kD,
                Extension.kS,
                Extension.kV,
                Extension.maxSpeed,
                Extension.maxAcceleration);
    }

    private TrapezoidProfile.Constraints createConstraints() {
        double maxVelRadPerSec = Extension.maxSpeed.get() * RADIANS_PER_METER;
        double maxAccelRadPerSecSq = Extension.maxAcceleration.get() * RADIANS_PER_METER;
        return new TrapezoidProfile.Constraints(maxVelRadPerSec, maxAccelRadPerSecSq);
    }

    private double clampRadians(double radians) {
        return MathUtil.clamp(radians, reverseLimitRad, forwardLimitRad);
    }

    private double radiansToMeters(double radians) {
        return radians * METERS_PER_RADIAN;
    }

    private void updateThermalModel(double totalStatorCurrent) {
        double perMotorCurrent = totalStatorCurrent / MOTOR_COUNT;
        double normalized = MathUtil.clamp(perMotorCurrent / Math.max(1.0, statorCurrentLimitPerMotor), 0.0, 2.0);
        double heatingPerSec = 12.0 * normalized;
        double coolingPerSec = (estimatedMotorTempC - AMBIENT_TEMP_C) * 0.35;
        estimatedMotorTempC += (heatingPerSec - coolingPerSec) * LOOP_PERIOD_SEC;
        estimatedMotorTempC = MathUtil.clamp(estimatedMotorTempC, AMBIENT_TEMP_C, MAX_TEMP_C);
    }
}
