package frc.robot.subsystems.drive.FastSwerve;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.CompetitionFieldUtils.Simulation.drive.Swerve.SwerveModuleSimulation;
import frc.robot.utils.CompetitionFieldUtils.Simulation.motorsims.SimulatedMotorController;
import frc.robot.utils.drive.DriveConstants;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController = new PIDController(.05, 0, 0);
    private final PIDController turnController = new PIDController(8.0, 0, 0);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(DriveConstants.kMaxDriveCurrent));
        this.turnMotor = moduleSimulation.useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(DriveConstants.kMaxTurnCurrent));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts
                    + driveController.calculate(
                            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                    moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

        // Update drive inputs
        inputs.driveMotorConnected = true;
        inputs.drivePositionRads = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadsPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveTorqueCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

        // Update turn inputs
        inputs.turnMotorConnected = true;
        inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnVelocityRadsPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnTorqueCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

        inputs.odometryDrivePositionsMeters = Arrays
                .stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(position -> position.baseUnitMagnitude()
                        * DriveConstants.TrainConstants.kWheelDiameter.get() / 2)
                .toArray();
        inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
    }

    @Override
    public void runDriveVolts(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void runCharacterization(double output) {
        runDriveVolts(output);
    }

    @Override
    public void runTurnVolts(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void runDriveVelocitySetpoint(double velocityRadPerSec, double feedForward) {
        driveClosedLoop = true;
        driveFFVolts = feedForward;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void runTurnPositionSetpoint(double angleRads) {
        double currentAngle = moduleSimulation.getSteerAbsoluteFacing().getRadians();
        double difference = angleRads - currentAngle;
        if (difference > Math.PI) {
            angleRads -= 2 * Math.PI;
        } else if (difference < -Math.PI) {
            angleRads += 2 * Math.PI;
        }
        runTurnVolts(turnController.calculate(currentAngle, angleRads));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD, double kS, double kV) {
        driveController.setPID(kP, kI, kD);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD, double kS, double kV, double deadbandAmps) {
        turnController.setPID(kP, kI, kD);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        // do nothing.
    }

    @Override
    public void stop() {
        runDriveVolts(0.0);
        runTurnVolts(0.0);
    }
}