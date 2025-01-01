package frc.robot.utils.leds;

import java.util.List;

public class LEDConstants {
	//Using doubles to prevent integer division
	public static double
	//The physical port where the LED strip is plugged in 
	ledPort = 9,
			//The number of LEDs in the PANELS (if there are multiple, they are daisy chained and MUST be the same dimensions)
			ledColsInFrame = 16, ledRowsInFrame = 16, ledColsPerPanel = 16, ledRowsPerPanel = 16,
			// amount of LEDs in the light trip
			ledBufferLength = 512;
	public static int[] bufferCutoffs = new int[]{256,310,350,390,410,430}; //The index of the first LED in each section, BESIDES the first section
	public static boolean[] sectionIsPanel = new boolean[]{true,false,false,false,false,false,false}; //Whether the section is a panel or a strip
	public static int[] noteRGB = new int[] { 255, 55, 10
	}, redRGB = new int[] { 255, 0, 0
	}, blueRGB = new int[] { 0, 0, 255
	}, greenRGB = new int[] { 0, 255, 0
	}, pinkRGB = new int[] { 255, 192, 203
	}, goldRGB = new int[] { 255, 215, 0
	}, disabledRGB = new int[] { 0, 0, 0
	}, whiteRGB = new int[] { 255, 255, 255
	};

	public enum LEDStates {
		OFF, DEBUG_PIXEL,SOLID_COLOR, RAINBOW, BLINK, PROGRESS, SINE_WAVE, WAVE2, BREATHING, GIF, FIRE, TEXT,STEPS
	}
	public enum PanelOrientation{
		TOP_RIGHT, TOP_LEFT, BOTTOM_RIGHT, BOTTOM_LEFT
	}
	public static PanelOrientation panelOrientation = PanelOrientation.BOTTOM_LEFT; //The orientation of ALL panels (MUST be same)
	//String constants
	public static String[] phrases = new String[]{
		"TOUCHDOWN", "GO PENN", "WIN!" 
	};
	//Wave constants
	public static double waveExponent = .5;

	public enum ImageStates {
		debug, gif1, gif2, logo
	}

	public static List<String> imageList = List.of(ImageStates.debug.name(),
			ImageStates.gif1.name(), ImageStates.gif2.name(),
			ImageStates.logo.name());
	//MUST MATCH ORDER IN IMAGESTATES ENUM
	public static List<List<byte[][]>> imageLedStates; //preprocessed on boot
}
