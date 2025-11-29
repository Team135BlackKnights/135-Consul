package frc.robot.subsystems.advancedMechs.dynamicElevator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.TuningConstants;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.advancedMechs.AdvancedMechanismConstants;
import frc.robot.utils.drive.EqualsUtil;
import frc.robot.utils.selfCheck.SelfChecking;
import lombok.Getter;
import lombok.Setter;

public class DynamicElevator extends SubsystemChecker {
    public enum ElevatorGoalState {
        DEFAULT(new LoggableTunedNumber("FeedForwardElevatorS/DefaultHeight", .0,TuningConstants.isTuningElevator)),
        INTAKING(new LoggableTunedNumber("FeedForwardElevatorS/Intaking",0.0+DEFAULT.getValue(),TuningConstants.isTuningElevator)),    
        L1_L2_GROUND(new LoggableTunedNumber("FeedForwardElevatorS/L2",
                .545+DEFAULT.getValue(),TuningConstants.isTuningElevator)), 
        L3(new LoggableTunedNumber("FeedForwardElevatorS/L3", .945+DEFAULT.getValue(),TuningConstants.isTuningElevator)),
        L4(new LoggableTunedNumber("FeedForwardElevatorS/L4", 1.55+DEFAULT.getValue(),TuningConstants.isTuningElevator)),//1.3625
        ONE_METER(new LoggableTunedNumber("FeedForwardElevatorS/OneMeter", 1,TuningConstants.isTuningElevator)),
        NET(new LoggableTunedNumber("FeedForwardElevatorS/Net",
                1.675,TuningConstants.isTuningElevator));

        private DoubleSupplier value;

        private ElevatorGoalState(LoggableTunedNumber value) {
            this.value = value::get;
        }

        public double getValue() {
            return value.getAsDouble();
        }

        public static ElevatorGoalState createElevatorGoalState(int x) {
            ElevatorGoalState ret = null;
            for (ElevatorGoalState type : ElevatorGoalState.values()) {
                if (type.getValue() == x)
                    ret = type;
            }
            return ret;
        }
    }

    private static final LoggableTunedNumber homingTime = new LoggableTunedNumber("DynamicElevator/HomingTime", .2,TuningConstants.isTuningElevator);
    private static final LoggableTunedNumber homingVolts = new LoggableTunedNumber("DynamicElevator/HomingVolts", -1.5,TuningConstants.isTuningElevator);
    private static final LoggableTunedNumber homingVelocityThresh = new LoggableTunedNumber(
            "DynamicElevator/HomingVelocityThresh", .5,TuningConstants.isTuningElevator);
    private static final LoggableTunedNumber tolerance = new LoggableTunedNumber("DynamicElevator/Tolerance", 0.2,TuningConstants.isTuningElevator);
    private static final LoggableTunedNumber staticCharacterizationVelocityThresh = new LoggableTunedNumber(
            "DynamicElevator/StaticCharacterizationVelocityThresh", 0.1,TuningConstants.isTuningElevator);
    private static final LoggableTunedNumber outputRampRate = new LoggableTunedNumber(
            "DynamicElevator/StaticCharacterizationOutputRampRate", 2,TuningConstants.isTuningElevator);
            private static final LoggableTunedNumber decreaseAccelHeight = new LoggableTunedNumber("DynamicElevator/DecreaseAccelHeight",.6,TuningConstants.isTuningElevator);
            private static final LoggableTunedNumber decreaseAccel = new LoggableTunedNumber("DynamicElevator/DecreaseAccel",4,TuningConstants.isTuningElevator);
    private final DynamicElevatorIO io;
    private final DynamicElevatorIOInputsAutoLogged inputs = new DynamicElevatorIOInputsAutoLogged();
    private BooleanSupplier coastOverride = () -> false;
    private BooleanSupplier disabledOverride = () -> false;
    private boolean hasGoneAboveMinSpeed = false;
    private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    @Getter
    private State setpoint = new State();
    private Supplier<State> goal = State::new;
    private State lastGoal = new State();
    private boolean stopProfile = false;
    @Getter
    private boolean shouldEStop = false;
    @Setter
    private boolean isEStopped = false;

    private double homedPosition = 0.0;

    @AutoLogOutput
    @Getter
    private boolean homed = false;
    private Debouncer homeDebouncer = new Debouncer(.1);
    private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private TrapezoidProfile slowProfile;
    private TrapezoidProfile fastProfile;
    @Getter
    @AutoLogOutput(key = "DynamicElevator/Profile/AtGoal")
    private boolean atGoal = false;
    public boolean tuning = false;

    public DynamicElevator(DynamicElevatorIO io) {
        this.io = io;

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        AdvancedMechanismConstants.DynamicElevator.maxVelocity.get(), AdvancedMechanismConstants.DynamicElevator.maxAccel.get()));
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("DynamicElevator", inputs);
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            homeDebouncer = new Debouncer(homingTime.get());
        }, homingTime);
        // Update tunable numbers
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            io.setPID(AdvancedMechanismConstants.DynamicElevator.kP.get(), AdvancedMechanismConstants.DynamicElevator.kI.get(),
                    AdvancedMechanismConstants.DynamicElevator.kD.get());
        }, AdvancedMechanismConstants.DynamicElevator.kP, AdvancedMechanismConstants.DynamicElevator.kD, AdvancedMechanismConstants.DynamicElevator.kI);
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            fastProfile = new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            AdvancedMechanismConstants.DynamicElevator.maxVelocity.get(),
                            AdvancedMechanismConstants.DynamicElevator.maxAccel.get()));
            slowProfile = new TrapezoidProfile(
                                new TrapezoidProfile.Constraints(
                                        AdvancedMechanismConstants.DynamicElevator.maxVelocity.get(),
                                        decreaseAccel.get()));
        }, AdvancedMechanismConstants.DynamicElevator.maxVelocity, AdvancedMechanismConstants.DynamicElevator.maxAccel, decreaseAccel);
        // Set coast mode
        setBrakeMode(!coastOverride.getAsBoolean());

        // Run profile
        final boolean shouldRunProfile = !stopProfile
                && !coastOverride.getAsBoolean()
                && !disabledOverride.getAsBoolean()
                && homed
                && !isEStopped
                && DriverStation.isEnabled();
        Logger.recordOutput("DynamicElevator/RunningProfile", shouldRunProfile);
        // Check if out of tolerance
        boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
        shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
        if (shouldRunProfile) {
            // Clamp goal
            var goalState = new State(
                    MathUtil.clamp(goal.get().position, 0.0, AdvancedMechanismConstants.DynamicElevator.maxPosition),
                    goal.get().velocity);
            //clamp accel profile
            if (lastGoal.position > .01 && goalState.position < 0 + decreaseAccelHeight.get()){
                if (getPositionMeters() < 0 + decreaseAccelHeight.get()){
                    profile = slowProfile;
                }else{

                    profile = fastProfile;
                }
            }else{
                profile = fastProfile;
            }
            setpoint = profile.calculate(.02, setpoint, goalState);
            if (setpoint.position < 0.0
                    || setpoint.position > AdvancedMechanismConstants.DynamicElevator.maxPosition) {
                setpoint = new State(
                        MathUtil.clamp(setpoint.position, 0.0, AdvancedMechanismConstants.DynamicElevator.maxPosition),
                        0.0);
            }
            double ff = AdvancedMechanismConstants.DynamicElevator.kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
                    + AdvancedMechanismConstants.DynamicElevator.kG.get();
            double posGoal = setpoint.position / AdvancedMechanismConstants.DynamicElevator.axleToMeters + homedPosition;
            double speedGoal = setpoint.velocity / AdvancedMechanismConstants.DynamicElevator.axleToMeters + homedPosition;
            Logger.recordOutput("DynamicElevator/Profile/AppliedFFVolts", ff);
            Logger.recordOutput("DynamicElevator/Profile/PosRad", posGoal);
            io.runPosition(
                    posGoal,
            speedGoal,
                    ff);
            /*
             * if (setpoint.position < .1){
             * if (!homed || (false)){
             * io.runVolts(homingVolts.get());
             * }
             * }
             */
            // Check at goal
            atGoal = EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
                    && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);
            if (atGoal){
                lastGoal = goalState;
            }

            // Log state
            Logger.recordOutput("DynamicElevator/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("DynamicElevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("DynamicElevator/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("DynamicElevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);    
        } else {
            // Reset setpoint
            setpoint = new State(getPositionMeters(), 0.0);

            // Clear logs
            Logger.recordOutput("DynamicElevator/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("DynamicElevator/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("DynamicElevator/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("DynamicElevator/Profile/GoalVelocityMetersPerSec", 0.0);
        }
        if (isEStopped) {
            io.stop();
        }

        // Log state
        /*Logger.recordOutput("DynamicElevator/CoastOverride", coastOverride.getAsBoolean());
        Logger.recordOutput("DynamicElevator/DisabledOverride", disabledOverride.getAsBoolean());
        Logger.recordOutput(
                "DynamicElevator/MeasuredVelocityMetersPerSec",
                inputs.velocityRadPerSec * StateSpaceConstants.Elevator.axleToMeters);*/
    }

    public void setGoal(DoubleSupplier goal) {
        setGoal(() -> new State(goal.getAsDouble(), 0.0));
    }

    public void setGoal(double goal) {
        if (homed) {
            setGoal(() -> new State(goal, 0.0));
        } else {
            DriverStation.reportWarning("Trying to set Elevator state while homing... Denying until homed.", false);
        }
    }

    public void setGoal(ElevatorGoalState goalState) {
        setGoal(goalState.getValue());
    }

    public void setGoal(Supplier<State> goal) {
        atGoal = false;
        this.goal = goal;
    }

    public boolean atGoal(double toleranceMeters) {
        boolean good = Math.abs(getPositionMeters()+ElevatorGoalState.DEFAULT.getValue() - goal.get().position) < toleranceMeters;
        Logger.recordOutput("DynamicElevator/SystemThinksGood", good);
        return good;
    }

    public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
        this.coastOverride = coastOverride;
        this.disabledOverride = disabledOverride;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public Command homingSequence() {
        if (io instanceof DynamicElevatorIOSim) {
            return Commands.runOnce(() -> {
                homedPosition = inputs.positionRad;
                homed = true;
                ((DynamicElevatorIOSim)io).setState(0, 0);
            });
        }
        if (Constants.currentMatchState == FRCMatchState.AUTO || Constants.currentMatchState == FRCMatchState.AUTOINIT || Constants.currentMatchState == FRCMatchState.DISABLED){
            return Commands.runOnce(() -> {
                homedPosition = inputs.positionRad;
                homed = true;
            });
        }
        return Commands.startRun(
                () -> {
                    //io.setCurrentLimit(100);
                    stopProfile = true;
                    homed = false;
                    hasGoneAboveMinSpeed = false;
                    homeDebouncer.calculate(false);
                    Logger.recordOutput("DynamicElevator/Homing/Zeroing", true);
                    Logger.recordOutput("DynamicElevator/Homing/CanZero", false);

                },
                () -> {
                    io.runVolts(homingVolts.get());
                    if (hasGoneAboveMinSpeed){
                        homed = homeDebouncer.calculate(Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
                    }else{
                        if (Math.abs(inputs.velocityRadPerSec) > homingVelocityThresh.get()){
                            hasGoneAboveMinSpeed = true;
                            Logger.recordOutput("DynamicElevator/Homing/CanZero", true);
                        }
                        if (Math.abs(inputs.torqueCurrentAmps) > 80){
                            //force home
                            homed = true;
                        }
                    }
                    
                })
                .until(() -> homed)
                .andThen(
                        () -> {
                            Logger.recordOutput("DynamicElevator/Homing/Zeroing", true);
                            homedPosition = inputs.positionRad;
                            homed = true;
                        })
                .finallyDo(
                        () -> {
                            io.setCurrentLimit(AdvancedMechanismConstants.DynamicElevator.currentLimit);
                            stopProfile = false;
                        });
    }

    public Command staticCharacterization() {
        final StaticCharacterizationState state = new StaticCharacterizationState();
        Timer timer = new Timer();
        return Commands.startRun(
                () -> {
                    stopProfile = true;
                    timer.restart();
                },
                () -> {
                    state.characterizationOutput = outputRampRate.get() * timer.get();
                    io.runOpenLoop(state.characterizationOutput);
                    Logger.recordOutput(
                            "DynamicElevator/StaticCharacterizationOutput", state.characterizationOutput);
                })
                .until(() -> inputs.velocityRadPerSec >= staticCharacterizationVelocityThresh.get())
                .finallyDo(
                        () -> {
                            stopProfile = false;
                            timer.stop();
                            Logger.recordOutput("DynamicElevator/CharacterizationOutput", state.characterizationOutput);
                        });
    }

    /** Get position of elevator in meters with 0 at home */
    @AutoLogOutput(key = "DynamicElevator/MeasuredHeightMeters")
    public double getPositionMeters() {
        return (inputs.positionRad - homedPosition) * AdvancedMechanismConstants.DynamicElevator.axleToMeters;
    }

    public double getGoalMeters() {
        return goal.get().position;
    }

    public double getError() {
        return getGoalMeters() - getPositionMeters();
    }

    private static class StaticCharacterizationState {
        public double characterizationOutput = 0.0;
    }

    @Override
    protected Command systemCheckCommand() {
        return //Commands.deadline(
            Commands
                .sequence(run(() -> setGoal(ElevatorGoalState.ONE_METER)).withTimeout(1.5), runOnce(() -> {
                    if (getError() > AdvancedMechanismConstants.DynamicElevator.elevatorSelfCheckMarginOfError) {
                        addFault(
                                "[System Check] Elevator did not reach position on time. Set one meter at "
                                        + Double.toString(ElevatorGoalState.ONE_METER.getValue()) + " meters, got "
                                        + Double.toString(getPositionMeters()),
                                false, true);
                    }
                }), run(() -> setGoal(ElevatorGoalState.L3)).withTimeout(1.5), runOnce(() -> {
                    if (getError() > AdvancedMechanismConstants.DynamicElevator.elevatorSelfCheckMarginOfError) {
                        addFault(
                                "[System Check] Elevator did not reach position on time. Set L3 level at "
                                        + Double.toString(ElevatorGoalState.L3.getValue()) + " meters, got "
                                        + Double.toString(getPositionMeters()),
                                false, true);
                    }
                })).until(() -> !getFaults().isEmpty())
                .andThen(runOnce(() -> setGoal(ElevatorGoalState.L1_L2_GROUND)));
               // RobotContainer.superStructure.setGoalCommand(Goal.TUNING_CHECKING));
    }

    @Override
    public List<ParentDevice> getOrchestraDevices() {
        List<ParentDevice> orchestra = new ArrayList<>();
        List<SelfChecking> driveHardware = io.getSelfCheckingHardware();
        for (SelfChecking motor : driveHardware) {
            if (motor.getHardware() instanceof TalonFX) {
                orchestra.add((TalonFX) motor.getHardware());
            }
        }
        return orchestra;
    }

    @Override
    public double getCurrent() {
        return inputs.torqueCurrentAmps;
    }

    @Override
    public HashMap<String, Double> getTemps() {
        HashMap<String, Double> temps = new HashMap<>();
        temps.put("ElevatorLeft", inputs.tempCelsius);
        return temps;
    }

    @Override
    public void setCurrentLimit(int amps) {
        DriverStation.reportWarning("Trying to set Dynamic Elevator current limit DYNAMICALLY?", false);
    }
}
