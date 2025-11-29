package frc.robot.subsystems.advancedMechs.dynamicElevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;

public class DynamicElevatorIOSim implements DynamicElevatorIO {
    private final ElevatorSim simElevator;
    private final PIDController controller = new PIDController(200, 0.0, 0.0);
    private double feedforward = 0.0, inputTorqueCurrent = 0.0;
    private Vector<N2> simState;
    private final DCMotor gearbox = DCMotor.getKrakenX60Foc(2);
    private double appliedVolts = 0.0;

    public DynamicElevatorIOSim() {
        simElevator = new ElevatorSim(gearbox,
                AdvancedMechanismConstants.DynamicElevator.elevatorGearing,
                AdvancedMechanismConstants.DynamicElevator.carriageMass,
                AdvancedMechanismConstants.DynamicElevator.axleToMeters,
                AdvancedMechanismConstants.DynamicElevator.elevatorMinHeight,
                AdvancedMechanismConstants.DynamicElevator.elevatorMaxHeight,
                false,
                AdvancedMechanismConstants.DynamicElevator.elevatorMinHeight);
        simState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    public void updateInputs(DynamicElevatorIOInputs inputs) {
        setInputTorqueCurrent(
                controller.calculate((simState.get(0) / AdvancedMechanismConstants.DynamicElevator.axleToMeters) - feedforward));
        update(.02);

        inputs.positionRad = simState.get(0) / AdvancedMechanismConstants.DynamicElevator.axleToMeters;
        inputs.velocityRadPerSec = simState.get(1) / AdvancedMechanismConstants.DynamicElevator.axleToMeters;
        inputs.appliedVolts = appliedVolts;
        inputs.torqueCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts);
        inputs.tempCelsius =  0.0;
    }

    @Override
    public void runOpenLoop(double output) {
        setInputTorqueCurrent(output);
    }

    @Override
    public void runVolts(double volts) {
        setInputVoltage(volts);
    }

    @Override
    public void stop() {
        runOpenLoop(0.0);
    }

    @Override
    public void runPosition(double positionRad, double velocity, double feedforward) {
        controller.setSetpoint(positionRad);
        this.feedforward = feedforward;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        controller.setPID(kP, kI, kD);
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        Logger.recordOutput("DynamicElevator/Sim/SimTorque", torqueCurrent);
        inputTorqueCurrent = torqueCurrent;
        appliedVolts = gearbox.getVoltage(
                gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0) / AdvancedMechanismConstants.DynamicElevator.axleToMeters);
        appliedVolts = MathUtil.clamp(appliedVolts, -6, 6);
    }

    private void setInputVoltage(double voltage) {
        setInputTorqueCurrent(gearbox.getCurrent(simState.get(1) / AdvancedMechanismConstants.DynamicElevator.axleToMeters, voltage));
    }
    public void setState(double positionMeters, double velocityMetersPerSecond){
        simElevator.setState(positionMeters, velocityMetersPerSecond);
    }
    private void update(double dt) {
        inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
        simElevator.setInputVoltage(appliedVolts);
        simElevator.update(dt);
        simState = VecBuilder.fill(simElevator.getPositionMeters(), simElevator.getVelocityMetersPerSecond());
    }

}
