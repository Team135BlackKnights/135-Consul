package frc.robot.subsystems.advancedMechs.PinkArm.wrist;

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
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants.PinkArm.Wrist;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.List;

public class WristIOSim implements WristIO {
    private enum ControlMode {
        IDLE,
        DUTY_CYCLE,
        POSITION
    }

    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double AMBIENT_TEMP_C = 28.0;
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
    private boolean brakeModeEnabled = true;
    private final double internalForwardLimitRad;
    private final double internalReverseLimitRad;
    private int statorCurrentLimitAmps = (int) Wrist.statorCurrentLimit;

    public WristIOSim() {
        internalForwardLimitRad = Wrist.maxPosition;
        internalReverseLimitRad = Wrist.startingPosition;

        SimMotorConfigs configs = new SimMotorConfigs(
                        DCMotor.getKrakenX44Foc(1),
                        Wrist.wristGearing,
                        KilogramSquareMeters.of(Wrist.wristMOI),
                        Volts.of(Wrist.kS.get()))
                .withHardLimits(Radians.of(internalForwardLimitRad), Radians.of(internalReverseLimitRad));

        motorSim = new MapleMotorSim(configs);
        motorController = motorSim.useSimpleDCMotorController();
        motorController.withCurrentLimit(Amps.of(Wrist.statorCurrentLimit));

        positionController = new PIDController(Wrist.kP.get(), Wrist.kI.get(), Wrist.kD.get());
        feedforward = new SimpleMotorFeedforward(Wrist.kS.get(), Wrist.kV.get());
        profileConstraints = createConstraints();

        double initialInternalPosition = motorSim.getAngularPosition().in(Radians);
        Rotation2d startingAngle = Rotation2d.fromDegrees(Wrist.startingPosition);
        sensorOffsetRadians = startingAngle.getRadians() - initialInternalPosition;

        goalState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
        setpointState = new TrapezoidProfile.State(initialInternalPosition, 0.0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        refreshTuningsIfNeeded();
        runControlStep();

        double mechanismPositionRad = motorSim.getAngularPosition().in(Radians);
        double mechanismVelocityRadPerSec = motorSim.getVelocity().in(RadiansPerSecond);
        double angularAcceleration = (mechanismVelocityRadPerSec - previousVelocityRadPerSec) / LOOP_PERIOD_SEC;
        previousVelocityRadPerSec = mechanismVelocityRadPerSec;

        double statorCurrent = motorSim.getStatorCurrent().in(Amps);
        updateThermalModel(Math.abs(statorCurrent));

        inputs.wristAngle = Rotation2d.fromRadians(mechanismPositionRad + sensorOffsetRadians);
        inputs.wristAppliedVolts = motorSim.getAppliedVoltage().in(Volts);
        inputs.wristSupplyCurrentAmps = motorSim.getSupplyCurrent().in(Amps);
        inputs.wristStatorCurrentAmps = statorCurrent;
        inputs.wristAngularVelocityRadPerSec = mechanismVelocityRadPerSec;
        inputs.wristAngularAccelerationRadPerSecSquared = angularAcceleration;
        inputs.wristMotorTemp = estimatedMotorTempC;
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        double internalTarget = clampToLimits(target.getRadians() - sensorOffsetRadians);
        goalState = new TrapezoidProfile.State(internalTarget, 0.0);
        setpointState = new TrapezoidProfile.State(
                motorSim.getAngularPosition().in(Radians),
                motorSim.getVelocity().in(RadiansPerSecond));
        controlMode = ControlMode.POSITION;
        dutyCycleCommand = 0.0;
    }

    @Override
    public void resetWristAngle(Rotation2d angle) {
        double actualPosition = motorSim.getAngularPosition().in(Radians);
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
    public void setBrakeMode(boolean enabled) {
        brakeModeEnabled = enabled;
    }

    @Override
    public void setCurrentLimit(int current) {
        statorCurrentLimitAmps = Math.max(1, current);
        motorController.withCurrentLimit(Amps.of(statorCurrentLimitAmps));
    }

    @Override
    public List<SelfChecking> getSelfCheckingHardware() {
        return List.of();
    }

    private void runControlStep() {
        switch (controlMode) {
            case POSITION -> runPositionControl();
            case DUTY_CYCLE -> runDutyCycleControl();
            case IDLE -> requestVoltage(brakeModeEnabled ? 0.0 : 0.0);
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
        motorController.requestVoltage(Volts.of(volts));
    }

    private void refreshTuningsIfNeeded() {
        LoggableTunedNumber.ifChanged(
                hashCode(),
                () -> {
                    positionController.setPID(Wrist.kP.get(), Wrist.kI.get(), Wrist.kD.get());
                    feedforward = new SimpleMotorFeedforward(Wrist.kS.get(), Wrist.kV.get());
                    profileConstraints = createConstraints();
                },
                Wrist.kP,
                Wrist.kI,
                Wrist.kD,
                Wrist.kS,
                Wrist.kV,
                Wrist.maxSpeed,
                Wrist.maxAcceleration);
    }

    private TrapezoidProfile.Constraints createConstraints() {
        double maxVelRadPerSec = Units.degreesToRadians(Wrist.maxSpeed.get());
        double maxAccelRadPerSecSq = Units.degreesToRadians(Wrist.maxAcceleration.get());
        return new TrapezoidProfile.Constraints(maxVelRadPerSec, maxAccelRadPerSecSq);
    }

    private double clampToLimits(double radians) {
        return MathUtil.clamp(radians, internalReverseLimitRad, internalForwardLimitRad);
    }

    private void updateThermalModel(double statorCurrent) {
        double normalized = MathUtil.clamp(statorCurrent / Math.max(1.0, statorCurrentLimitAmps), 0.0, 2.0);
        double heatingPerSec = 18.0 * normalized;
        double coolingPerSec = (estimatedMotorTempC - AMBIENT_TEMP_C) * 0.4;
        estimatedMotorTempC += (heatingPerSec - coolingPerSec) * LOOP_PERIOD_SEC;
        estimatedMotorTempC = MathUtil.clamp(estimatedMotorTempC, AMBIENT_TEMP_C, MAX_TEMP_C);
    }
}
