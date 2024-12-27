package frc.robot.utils;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class DriverStationHID {
    private final GenericHID HID;
    private DSLEDPattern currentLEDPattern = DSLEDPattern.LEDS_OFF;
    /**
     * Create a new DriverStationHID controller (combination button board and led
     * handler). Button indexes begin at 1.
     * 
     * @param port the port that the controller is plugged into
     */
    public DriverStationHID(int port) {
        this.HID = new GenericHID(port);
    }

    /**
     * Get the button state
     * 
     * @param button the id of the button to check
     * @return whether the button is pressed
     */
    public boolean getButtonState(int button) {
        return HID.getRawButton(button);
    }

    /**
     * Get if the button was pressed since last check
     * 
     * @param button the id of the button to check
     * @return whether the button is pressed since last check
     */
    public boolean getRawButtonPressed(int button) {
        return HID.getRawButtonPressed(button);
    }

    /**
     * Get if the button was released since last check
     * 
     * @param button the id of the button to check
     * @return whether the button is released since last check
     */
    public boolean getRawButtonReleased(int button) {
        return HID.getRawButtonReleased(button);
    }

    /**
     * Constructs an event instance around this button's digital signal.
     *
     * @param button the button index
     * @param loop   the event loop instance to attach the event to.
     * @return an event instance representing the button's digital signal attached
     *         to the given loop.
     */
    public BooleanEvent button(int button, EventLoop loop) {
        return HID.button(button, loop);
    }
    /** Set the lights on the DS LED Controller to a pattern
     * 
     * @param pattern the LED Pattern to run
     */
    public void setDSLEDPattern(DSLEDPattern pattern){
        HID.setOutput(pattern.value, true);
        currentLEDPattern = pattern;
    }
    public DSLEDPattern getCurrentLEDPattern(){
        return currentLEDPattern;
    }
    // This needs to be updated to match the functions inside
    // https://github.com/Team135BlackKnights/DriverStationHID/commits/main/
    public enum DSLEDPattern {
        LEDS_OFF(0),
        RAINBOW(1),
        LEDS_GOLD(2),
        LEDS_SILVER(3),
        BREATHING_GOLD(4);

        @SuppressWarnings("unused")
        private final int value;

        DSLEDPattern(final int newValue) {
            value = newValue;
        }

    }
}
