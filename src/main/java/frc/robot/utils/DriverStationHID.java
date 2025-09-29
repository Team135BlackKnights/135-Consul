package frc.robot.utils;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class DriverStationHID extends GenericHID implements Sendable {
    /** Represents a digital button on a XboxController. */
    public enum Button {
        /** Coral branch 1 */
        branch1Button(2), //23
        /** Coral branch 2 */
        branch2Button(4), //25
        /** Coral branch 3 */
        branch3Button(6), //27
        /** Coral branch 4 */
        branch4Button(8), //29
        /** Coral branch 5 */
        branch5Button(10), //31
        /** Coral branch 6 */
        branch6Button(12), //33
        /** Coral branch 7 */
        branch7Button(14), //32
        /** Coral branch 8 */
        branch8Button(1), //22
        /** Coral branch 9 */
        branch9Button(3), //24
        /** Coral branch 10 */
        branch10Button(5), //26
        /** Coral branch 11 */
        branch11Button(7), //28
        /** Coral branch 12 */
        branch12Button(9), //30
        /** Intake algae button */
        intakeAlgaeButton(16), //34
        /** Go to processor button */
        goToProcessorButton(18), //35
        /** Score at the processor button */
        scoreProcessorButton(20), //36

        resetCoralButton(11), //also intakes
        /**Go to processor button*/
        runButton(22), //37
        /** Manual elevator Up. */
        manualElevatorUp(29), //38
        /** Manual elevator down. */
        manualElevatorDown(29), //39
        /** Manual arm up */
        manualArmUp(29), //41
        /** Manual arm down */
        manualArmDown(29), //40
        /** Activate manual elevator */
        activateManualElevator(15), //42
        /** Activate manual arm */
        activateManualArm(17), //43
        /** Currently unused, left one on DS */
        reefAlgae(19), //44
        /** Currently unused, right switch on DS */
        unusedRight(21), //45
        /** Level 1/2 Button. */
        level1and2Button(25), //46 
        /** Level 3 Button. */
        level3Button(27), //not a real pin!
        /** Level 4 Button. */
        level4Button(26); //47
        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            // Remove leading `k`
            return this.name().substring(1) + "Button";
        }
    }
    

    private DSLEDPattern currentLEDPattern = DSLEDPattern.LEDS_OFF;
    private boolean[] ledStates = {false, false, false};

        /**
     * Set the lights on the DS LED Controller to a pattern
     * 
     * @param pattern the LED Pattern to run
     */
    public void setDSLEDPattern(DSLEDPattern pattern) {
        currentLEDPattern = pattern;
    }
    public void updateDSHIDLEDS(){
        int ledStateValue = 0;
        for (int i = 0; i < ledStates.length; i++){
            if (ledStates[i]){
                ledStateValue += Math.pow(2, i);
            }
        }
        super.setOutputs(currentLEDPattern.value+ledStateValue);
        
    }

    public DSLEDPattern getCurrentLEDPattern() {
        return currentLEDPattern;
    }
    //Protocol definition: Use first two bits to update the led pattern, use second two to update which LEDs we should have on
    // This needs to be updated to match the functions inside
    // https://github.com/Team135BlackKnights/DriverStationHID/commits/main/
    public enum DSLEDPattern {
        LEDS_OFF(0x0000),
        RAINBOW(0x0100),
        LEDS_GOLD(0x0200),
        LEDS_SILVER(0x0300),
        BREATHING_GOLD(0x0400);

        private final int value;

        DSLEDPattern(final int newValue) {
            value = newValue;
        }

    }
    /**
     * Sets an individual LED on the button board
     * @param index the id of the led (id 0-3)
     * @param value on or off (true or false)
     */
    public void setIndividualLED(int index, boolean value){
        ledStates[index] = value;
    }
    public boolean[] getIndividualLEDStates(){
        return ledStates;
    }


    /**
     * Create a new DriverStationHID controller (combination button board and led
     * handler). Button indexes begin at 1.
     * 
     * @param port the port that the controller is plugged into
     */
    public DriverStationHID(int port) {
        super(port);
    }

    /**
     * Read the value of the branch 1 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch1Button() {
        return getRawButton(Button.branch1Button.value);
    }

    /**
     * Whether the branch 1 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch1ButtonPressed() {
        return getRawButtonPressed(Button.branch1Button.value);
    }

    /**
     * Whether the branch 1 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch1ButtonReleased() {
        return getRawButtonReleased(Button.branch1Button.value);
    }

    /**
     * Constructs an event instance around the branch 1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 1 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch1Button(EventLoop loop) {
        return button(Button.branch1Button.value, loop);
    }

        /**
     * Read the value of the branch 2 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch2Button() {
        return getRawButton(Button.branch2Button.value);
    }

    /**
     * Whether the branch 2 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch2ButtonPressed() {
        return getRawButtonPressed(Button.branch2Button.value);
    }

    /**
     * Whether the branch 2 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch2ButtonReleased() {
        return getRawButtonReleased(Button.branch2Button.value);
    }

    /**
     * Constructs an event instance around the branch 2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 2 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch2Button(EventLoop loop) {
        return button(Button.branch2Button.value, loop);
    }
        /**
     * Read the value of the branch 3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch3Button() {
        return getRawButton(Button.branch3Button.value);
    }

    /**
     * Whether the branch 3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch3ButtonPressed() {
        return getRawButtonPressed(Button.branch3Button.value);
    }

    /**
     * Whether the branch 3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch3ButtonReleased() {
        return getRawButtonReleased(Button.branch3Button.value);
    }

    /**
     * Constructs an event instance around the branch 3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 3 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch3Button(EventLoop loop) {
        return button(Button.branch3Button.value, loop);
    }
        /**
     * Read the value of the branch 4 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch4Button() {
        return getRawButton(Button.branch4Button.value);
    }

    /**
     * Whether the branch 4 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch4ButtonPressed() {
        return getRawButtonPressed(Button.branch4Button.value);
    }

    /**
     * Whether the branch 4 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch4ButtonReleased() {
        return getRawButtonReleased(Button.branch4Button.value);
    }

    /**
     * Constructs an event instance around the branch 4 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 4 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch4Button(EventLoop loop) {
        return button(Button.branch4Button.value, loop);
    
    }
        /**
     * Read the value of the branch 5 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch5Button() {
        return getRawButton(Button.branch5Button.value);
    }

    /**
     * Whether the branch 5 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch5ButtonPressed() {
        return getRawButtonPressed(Button.branch5Button.value);
    }

    /**
     * Whether the branch 5 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch5ButtonReleased() {
        return getRawButtonReleased(Button.branch5Button.value);
    }

    /**
     * Constructs an event instance around the branch 5 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 5 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch5Button(EventLoop loop) {
        return button(Button.branch5Button.value, loop);
    }
        /**
     * Read the value of the branch 6 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch6Button() {
        return getRawButton(Button.branch6Button.value);
    }

    /**
     * Whether the branch 6 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch6ButtonPressed() {
        return getRawButtonPressed(Button.branch6Button.value);
    }

    /**
     * Whether the branch 6 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch6ButtonReleased() {
        return getRawButtonReleased(Button.branch6Button.value);
    }

    /**
     * Constructs an event instance around the branch 6 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 6 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch6Button(EventLoop loop) {
        return button(Button.branch6Button.value, loop);
    }
        /**
     * Read the value of the branch 7 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch7Button() {
        return getRawButton(Button.branch7Button.value);
    }

    /**
     * Whether the branch 7 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch7ButtonPressed() {
        return getRawButtonPressed(Button.branch7Button.value);
    }

    /**
     * Whether the branch 7 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch7ButtonReleased() {
        return getRawButtonReleased(Button.branch7Button.value);
    }

    /**
     * Constructs an event instance around the branch 7 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 7 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch7Button(EventLoop loop) {
        return button(Button.branch7Button.value, loop);
    }
        /**
     * Read the value of the branch 8 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch8Button() {
        return getRawButton(Button.branch8Button.value);
    }

    /**
     * Whether the branch 8 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch8ButtonPressed() {
        return getRawButtonPressed(Button.branch8Button.value);
    }

    /**
     * Whether the branch 8 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch8ButtonReleased() {
        return getRawButtonReleased(Button.branch8Button.value);
    }

    /**
     * Constructs an event instance around the branch 8 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 8 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch8Button(EventLoop loop) {
        return button(Button.branch8Button.value, loop);
    }
        /**
     * Read the value of the branch 9 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch9Button() {
        return getRawButton(Button.branch9Button.value);
    }

    /**
     * Whether the branch 9 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch9ButtonPressed() {
        return getRawButtonPressed(Button.branch9Button.value);
    }

    /**
     * Whether the branch 9 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch9ButtonReleased() {
        return getRawButtonReleased(Button.branch9Button.value);
    }

    /**
     * Constructs an event instance around the branch 9 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 9 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch9Button(EventLoop loop) {
        return button(Button.branch9Button.value, loop);
    }
        /**
     * Read the value of the branch 10 on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch10Button() {
        return getRawButton(Button.branch10Button.value);
    }

    /**
     * Whether the branch 10 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch10ButtonPressed() {
        return getRawButtonPressed(Button.branch10Button.value);
    }

    /**
     * Whether the branch 10 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch10ButtonReleased() {
        return getRawButtonReleased(Button.branch10Button.value);
    }

    /**
     * Constructs an event instance around the branch 10 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 10 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch10Button(EventLoop loop) {
        return button(Button.branch10Button.value, loop);
    }
    /**
     * Read the value of the branch 11 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch11Button() {
        return getRawButton(Button.branch11Button.value);
    }

    /**
     * Whether the branch 11 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch11ButtonPressed() {
        return getRawButtonPressed(Button.branch11Button.value);
    }

    /**
     * Whether the branch 11 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch11ButtonReleased() {
        return getRawButtonReleased(Button.branch11Button.value);
    }

    /**
     * Constructs an event instance around the branch 11 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 11 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch11Button(EventLoop loop) {
        return button(Button.branch11Button.value, loop);
    }
    /**
     * Read the value of the branch 12 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBranch12Button() {
        return getRawButton(Button.branch12Button.value);
    }

    /**
     * Whether the branch 12 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBranch12ButtonPressed() {
        return getRawButtonPressed(Button.branch12Button.value);
    }

    /**
     * Whether the branch 12 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBranch12ButtonReleased() {
        return getRawButtonReleased(Button.branch12Button.value);
    }

    /**
     * Constructs an event instance around the branch 12 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 12 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent branch12Button(EventLoop loop) {
        return button(Button.branch12Button.value, loop);
    }
        /**
     * Read the value of the algae intake button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getIntakeAlgaeButton() {
        return getRawButton(Button.intakeAlgaeButton.value);
    }

    /**
     * Whether the algae intake button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getIntakeAlgaeButtonPressed() {
        return getRawButtonPressed(Button.intakeAlgaeButton.value);
    }

    /**
     * Whether the algae intake button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getIntakeAlgaeButtonReleased() {
        return getRawButtonReleased(Button.intakeAlgaeButton.value);
    }

    /**
     * Constructs an event instance around the algae intake button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the algae intake button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent intakeAlgaeButton(EventLoop loop) {
        return button(Button.intakeAlgaeButton.value, loop);
    }

        /**
     * Read the value of the score processor button on the controller.
     *
     * @return The state of the button.
     */

     public boolean getScoreProcessorButton() {
        return getRawButton(Button.scoreProcessorButton.value);
    }

    /**
     * Whether the score processor button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getScoreProcessorButtonPressed() {
        return getRawButtonPressed(Button.scoreProcessorButton.value);
    }

    /**
     * Whether the score processor button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getScoreProcessorButtonReleased() {
        return getRawButtonReleased(Button.scoreProcessorButton.value);
    }

    /**
     * Constructs an event instance around the score processor's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the score processor button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent scoreProcessorButton(EventLoop loop) {
        return button(Button.scoreProcessorButton.value, loop);
    }
    
        /**
     * Read the value of the go to processor button on the controller.
     *
     * @return The state of the button.
     */

     public boolean getGoToProcessorButton() {
        return getRawButton(Button.goToProcessorButton.value);
    }

    /**
     * Whether the go to processor button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getGoToProcessorButtonPressed() {
        return getRawButtonPressed(Button.goToProcessorButton.value);
    }

    /**
     * Whether the go to processor button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getGoToProcessorButtonReleased() {
        return getRawButtonReleased(Button.goToProcessorButton.value);
    }

    /**
     * Constructs an event instance around the go to processor's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the go to processor button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent GoToProcessorButton(EventLoop loop) {
        return button(Button.goToProcessorButton.value, loop);
    }

        /**
     * Read the value of the run button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRunButton() {
        return getRawButton(Button.runButton.value);
    }

    /**
     * Whether the run button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRunButtonPressed() {
        return getRawButtonPressed(Button.runButton.value);
    }

    /**
     * Whether the run button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRunButtonReleased() {
        return getRawButtonReleased(Button.runButton.value);
    }

    /**
     * Constructs an event instance around the run button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the run button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent runButton(EventLoop loop) {
        return button(Button.runButton.value, loop);
    }
        /**
     * Read the value of the level 1 and 2 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel1and2Button() {
        return getRawButton(Button.level1and2Button.value);
    }

    /**
     * Whether the level 1 and 2 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel1and2ButtonPressed() {
        return getRawButtonPressed(Button.level1and2Button.value);
    }

    /**
     * Whether the level 1 and 2 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel1and2ButtonReleased() {
        return getRawButtonReleased(Button.level1and2Button.value);
    }

    /**
     * Constructs an event instance around the level 1 and 2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 1 and 2 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent level1And2Button(EventLoop loop) {
        return button(Button.level1and2Button.value, loop);
    }
        /**
     * Read the value of the level 3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel3Button() {
        return getRawButton(Button.level3Button.value);
    }

    /**
     * Whether the level 3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel3ButtonPressed() {
        return getRawButtonPressed(Button.level3Button.value);
    }

    /**
     * Whether the level 3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel3ButtonReleased() {
        return getRawButtonReleased(Button.level3Button.value);
    }

    /**
     * Constructs an event instance around the level 3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 3 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent level3Button(EventLoop loop) {
        return button(Button.level3Button.value, loop);
    }
        /**
     * Read the value of the level 4 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLevel4Button() {
        return getRawButton(Button.level4Button.value);
    }

    /**
     * Whether the level 4 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLevel4ButtonPressed() {
        return getRawButtonPressed(Button.level4Button.value);
    }

    /**
     * Whether the level 4 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLevel4ButtonReleased() {
        return getRawButtonReleased(Button.level4Button.value);
    }

    /**
     * Constructs an event instance around the level 4 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the level 4 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent level4Button(EventLoop loop) {
        return button(Button.level4Button.value, loop);
    }
        /**
     * Read the value of the manual elevator up button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getManualElevatorUp() {
        return getRawButton(Button.manualElevatorUp.value);
    }

    /**
     * Whether the manual elevator up button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getManualElevatorUpPressed() {
        return getRawButtonPressed(Button.manualElevatorUp.value);
    }

    /**
     * Whether the manual elevator up button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getManualElevatorUpReleased() {
        return getRawButtonReleased(Button.manualElevatorUp.value);
    }

    /**
     * Constructs an event instance around the manual elevator up button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the manual elevator up button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent manualElevatorUp(EventLoop loop) {
        return button(Button.manualElevatorUp.value, loop);
    }
        /**
     * Read the value of the manual elevator down button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getManualElevatorDown() {
        return getRawButton(Button.manualElevatorDown.value);
    }

    /**
     * Whether the manual elevator down button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getManualElevatorDownPressed() {
        return getRawButtonPressed(Button.manualElevatorDown.value);
    }

    /**
     * Whether the manual elevator down button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getManualElevatorDownReleased() {
        return getRawButtonReleased(Button.manualElevatorDown.value);
    }

    /**
     * Constructs an event instance around the manual elevator down button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the manual elevator down button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent manualElevatorDown(EventLoop loop) {
        return button(Button.manualElevatorDown.value, loop);
    }
        /**
     * Read the value of the manual arm up button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getManualArmUp() {
        return getRawButton(Button.manualArmUp.value);
    }

    /**
     * Whether the manual arm up button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getManualArmUpPressed() {
        return getRawButtonPressed(Button.manualArmUp.value);
    }

    /**
     * Whether the manual arm up button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getManualArmUpReleased() {
        return getRawButtonReleased(Button.manualArmUp.value);
    }

    /**
     * Constructs an event instance around the manual arm up button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the manual arm up button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent manualArmUp(EventLoop loop) {
        return button(Button.branch1Button.value, loop);
    }
        /**
     * Read the value of the manual arm down button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getManualArmDown() {
        return getRawButton(Button.manualArmDown.value);
    }

    /**
     * Whether the manual arm down button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getManualArmDownPressed() {
        return getRawButtonPressed(Button.manualArmDown.value);
    }

    /**
     * Whether the manual arm down button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getManualArmDownReleased() {
        return getRawButtonReleased(Button.manualArmDown.value);
    }

    /**
     * Constructs an event instance around the manual arm down button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the manual arm down button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent manualArmDown(EventLoop loop) {
        return button(Button.manualArmDown.value, loop);
    }
        /**
     * Read the value of the activate manual arm switch on the controller.
     *
     * @return The state of the switch.
     */

     public boolean getActivateManualArmSwitch() {
        return getRawButton(Button.activateManualArm.value);
    }

    /**
     * Whether the activate manual arm switch was pressed since the last check.
     *
     * @return Whether the switch was pressed since the last check.
     */
    public boolean getActivateManualArmSwitchPressed() {
        return getRawButtonPressed(Button.activateManualArm.value);
    }

    /**
     * Whether the activate manual arm switch was released since the last check.
     *
     * @return Whether the switch was released since the last check.
     */
    public boolean getActivateManualArmSwitchReleased() {
        return getRawButtonReleased(Button.activateManualArm.value);
    }

    /**
     * Constructs an event instance around the activate manual arm switch's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the activate manual arm's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent activateManualArmSwitch(EventLoop loop) {
        return button(Button.activateManualArm.value, loop);
    }

        /**
     * Read the value of the activate manual elevator switch on the controller.
     *
     * @return The state of the switch.
     */

     public boolean getActivateManualElevatorSwitch() {
        return getRawButton(Button.activateManualElevator.value);
    }

    /**
     * Whether the activate manual elevator switch was pressed since the last check.
     *
     * @return Whether the switch was pressed since the last check.
     */
    public boolean getActivateManualElevatorSwitchPressed() {
        return getRawButtonPressed(Button.activateManualElevator.value);
    }

    /**
     * Whether the activate manual elevator was released since the last check.
     *
     * @return Whether the activate manual elevator switch was released since the last check.
     */
    public boolean getActivateManualElevatorSwitchReleased() {
        return getRawButtonReleased(Button.activateManualElevator.value);
    }

    /**
     * Constructs an event instance around the manual elevator switch's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the manual elevator switch's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent activateManualElevatorSwitch(EventLoop loop) {
        return button(Button.activateManualElevator.value, loop);
    }

        /**
     * Read the value of the reef algae switch on the controller.
     *
     * @return The state of the switch.
     */

     public boolean getReefAlgaeSwitch() {
        return getRawButton(Button.reefAlgae.value);
    }

    /**
     * Whether the Reef Algae switch was pressed since the last check.
     *
     * @return Whether the switch was pressed since the last check.
     */
    public boolean getReefAlgaeSwitchPressed() {
        return getRawButtonPressed(Button.reefAlgae.value);
    }

    /**
     * Whether the Reef Algae switch was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getReefAlgaeSwitchReleased() {
        return getRawButtonReleased(Button.reefAlgae.value);
    }

    /**
     * Constructs an event instance around the reef algae switch's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the reef algae switch's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent reefAlgaeSwitch(EventLoop loop) {
        return button(Button.reefAlgae.value, loop);
    }
        /**
     * Read the value of the unused right switch on the controller.
     *
     * @return The state of the switch.
     */

     public boolean getUnusedRightSwitch() {
        return getRawButton(Button.unusedRight.value);
    }

    /**
     * Whether the unused right switch was pressed since the last check.
     *
     * @return Whether the switch was pressed since the last check.
     */
    public boolean getUnusedRightSwitchPressed() {
        return getRawButtonPressed(Button.unusedRight.value);
    }

    /**
     * Whether the unused right switch was released since the last check.
     *
     * @return Whether the switch was released since the last check.
     */
    public boolean getUnusedRightSwitchReleased() {
        return getRawButtonReleased(Button.unusedRight.value);
    }

    /**
     * Constructs an event instance around the unused right switch's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the unused right switch's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent unusedRightSwitch(EventLoop loop) {
        return button(Button.unusedRight.value, loop);
    }

    /**
     * Read the value of the branch 3 button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getResetReef() {
        return getRawButton(Button.resetCoralButton.value);
    }

    /**
     * Whether the branch 3 button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getResetReefPressed() {
        return getRawButtonPressed(Button.resetCoralButton.value);
    }

    /**
     * Whether the branch 3 button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getResetReefReleased() {
        return getRawButtonReleased(Button.resetCoralButton.value);
    }

    /**
     * Constructs an event instance around the branch 3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the branch 3 button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent resetReefButton(EventLoop loop) {
        return button(Button.resetCoralButton.value, loop);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.publishConstString("ControllerType", "2025FRCButtonBoard135");
        builder.addBooleanProperty("Branch1Button", this::getBranch1Button, null);
        builder.addBooleanProperty("Branch2Button", this::getBranch2Button, null);
        builder.addBooleanProperty("Branch3Button", this::getBranch3Button, null);
        builder.addBooleanProperty("Branch4Button", this::getBranch4Button, null);
        builder.addBooleanProperty("Branch5Buttons",this::getBranch5Button, null);
        builder.addBooleanProperty("Branch6Button", this::getBranch6Button, null);
        builder.addBooleanProperty("Branch7Button", this::getBranch7Button, null);
        builder.addBooleanProperty("Branch8Button", this::getBranch8Button, null);
        builder.addBooleanProperty("Branch9Button", this::getBranch9Button, null);
        builder.addBooleanProperty("Branch10Button", this::getBranch10Button, null);
        builder.addBooleanProperty("Branch11Button", this::getBranch11Button, null);
        builder.addBooleanProperty("Branch12Button", this::getBranch12Button, null);
        builder.addBooleanProperty("IntakeAlgaeButton", this::getIntakeAlgaeButton, null);
        builder.addBooleanProperty("GoToAlgaeButton", this::getGoToProcessorButton, null);
        builder.addBooleanProperty("ScoreProcessorButton", this::getScoreProcessorButton, null);
        builder.addBooleanProperty("RunButton", this::getRunButton, null);
        builder.addBooleanProperty("Level1And2Button", this::getLevel1and2Button, null);
        builder.addBooleanProperty("Level3Button", this::getLevel3Button, null);
        builder.addBooleanProperty("Level4Button", this::getLevel4Button, null);
        builder.addBooleanProperty("ManualElevatorUp", this::getManualElevatorUp, null);
        builder.addBooleanProperty("ManualElevatorDown", this::getManualElevatorDown, null);
        builder.addBooleanProperty("ManualArmUp", this::getManualArmUp, null);
        builder.addBooleanProperty("ManualArmDown", this::getManualArmDown, null);
        builder.addBooleanProperty("ActivateElevatorSwitch", this::getActivateManualElevatorSwitch, null);
        builder.addBooleanProperty("ManualArmDown", this::getActivateManualArmSwitch, null);
        builder.addBooleanProperty("ReefAlgae", this::getReefAlgaeSwitch, null);
        builder.addBooleanProperty("Unused", this::getUnusedRightSwitch, null);
        builder.addBooleanProperty("ReefCoral", this::getResetReef, null);
        builder.addBooleanArrayProperty("IndividualLEDStates", this::getIndividualLEDStates, null);
        builder.addStringProperty("LEDPattern", () -> currentLEDPattern.toString(), null);
    }
}
