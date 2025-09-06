package frc.robot;

import java.io.BufferedReader;
import java.io.File;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.maths.TimeUtil;

import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.BindException;
import java.net.ServerSocket;
import java.net.Socket;

import java.lang.reflect.Type;

/**
 * Class used for logging data. Coded as an alternative to DataLogManager (so
 * you don't get every update in NetworkTables logged, just the values you
 * want), designed to be used for polynomial regression. Saves data in columns
 * instead of rows (each value for x is represented by the values below it) Used
 * in conjunction with the 135 PyDriverStation to generate accurate models.
 */
public class DataHandler {
	public static boolean isUSBConnected = true;
	private static Gson gson;
	static Map<String, String> responseData = new HashMap<>();
	private static int port = 5802;
	private static ServerSocket serverSocket;
	private static final boolean usingLaptop = true;
	private static int oldTime = 0;
	private static long oldTimestamp = 0; 
	/**
	 * Call this in Robot.java. Starts the handler and has contingencies to use
	 * the NetworkTables, write to usb, or write to a sim disk drive
	 * 
	 * @param useNetworkTables A boolean that states whether to write to a
	 *                            physical USB or use the networktables
	 * @param simDiskName      A string that states what disk to write to in
	 *                            simulation
	 */
	public static void startHandler() {
		gson = new Gson();
		try {
			while (serverSocket == null) {
				try {
					serverSocket = new ServerSocket(port);
				}
				catch (BindException e) {
					System.err.println("Port " + port
							+ " is already in use. Trying next port...");
					port++; // Try the next port
				}
			}
			System.err.println(port + "GOOD!");
			if (Constants.currentMode == Constants.Mode.SIM) {
				PortForwarder.add(port, "localhost", port);
			} else {
				PortForwarder.add(port, "10.1.35.2", port);
			}
			serverSocket.setSoTimeout(1);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		//Start the handler
		Thread dataHandlerThread = new Thread(new Runnable() {
			@Override
			public void run() {
				while (true) { // Loop to keep the thread running continuously
					updateHandlerState();
					try {
						Thread.sleep(20); // Sleep to prevent overloading the CPU
					}
					catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		});
		dataHandlerThread.setPriority(3);
		dataHandlerThread.setName("DataHandler");
		dataHandlerThread.setDaemon(true);
		dataHandlerThread.start();
	}

	/**
	 * Writes values to file and sends them through networkTables Recommended to
	 * start by logging the table heading names first, polynomial regression tool
	 * handles this. If there is a time when you want to log data but want to
	 * ignore something, put null in as the value in the array (will output a
	 * string "null"). Regression calculator currently cannot handle this
	 * exception, as well as data relationships that have more than 2 variables
	 * (y = f(x) type functions) Writes everything as a string, please convert
	 * values to strings before adding them to the array.
	 * 
	 * @param tableHeadings the array of values to be logged, can be different
	 *                         from the values declared in the setUpLogOnUsb
	 */
	public static void logData(String data, String key) {
		responseData.put(key, data); //send to network tables
		Logger.recordOutput("customLogger/" + key, data);
	}

	public static void logData(String[] data, String key) {
		//String that will be output to the writer
		String lineToBeSaved = "";
		//Adds each argument in the array to the string, adds a comma for separation (regression calculator uses this as well)
		for (String heading : data) {
			lineToBeSaved += (heading + ",");
		}
		//Removes last comma at the end
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(int data, String key) {
		String dataString = Integer.toString(data);
		logData(dataString, key);
	}

	public static void logData(int[] data, String key) {
		String lineToBeSaved = "";
		for (int integer : data) {
			lineToBeSaved += (Integer.toString(integer) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(boolean data, String key) {
		String dataString = Boolean.toString(data);
		logData(dataString, key);
	}

	public static void logData(boolean[] data, String key) {
		String lineToBeSaved = "";
		for (boolean bool : data) {
			lineToBeSaved += (Boolean.toString(bool) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(double data, String key) {
		String dataString = Double.toString(data);
		logData(dataString, key);
	}

	public static void logData(double[] data, String key) {
		String lineToBeSaved = "";
		for (double num : data) {
			lineToBeSaved += (Double.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(List<Double> data, String key) {
		String lineToBeSaved = "";
		for (double num : data) {
			lineToBeSaved += (Double.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	public static void logData(float data, String key) {
		String dataString = Float.toString(data);
		logData(dataString, key);
	}

	public static void logData(float[] data, String key) {
		String lineToBeSaved = "";
		for (float num : data) {
			lineToBeSaved += (Float.toString(num) + ",");
		}
		lineToBeSaved = lineToBeSaved.substring(0, (lineToBeSaved.length() - 1));
		logData(lineToBeSaved, key);
	}

	/**
	 * Checks the USB connection status of the RIO by making sure the directory
	 * still exists
	 */
	public static void pingUSB() {
		isUSBConnected = new File("U/Logs/").exists();
	}

	/**
	 * Updates the state of the handler, and checks if the USB has been
	 * disconnected. Call this in the periodic function of the file you called
	 * createNewWriter in.
	 */
	static double previousTime = 0;
	static String oldModel = "";
	static String[] outputList;

	/**
	 * Get any value from the output model from Python.
	 * 
	 * @param index ZERO AS FIRST OUTPUT
	 * @return IN A DOUBLE your selected index for the model outputs.
	 */
	public static double getValue(int index) {
		return Double.parseDouble(outputList[index]);
	}

	private static List<Double> makeDoubleList(String data) {
		String formattedData = data.replace("[", "").replace("]", "").trim();
		String[] outputs = formattedData.split("\\s+");
		List<Double> outputList = new ArrayList<>();
		for (String numberString : outputs) {
			outputList.add(Double.parseDouble(numberString));
		}
		return outputList;
	}

	/**
	 * Updates state of the handler, and continually sends any data via network
	 * tables. Whenever we have a change in NetworkTables, log that as well.q
	 */
	public static void updateHandlerState() {
		String dataHandlerJson = SmartDashboard.getString("ToRobot", "default");
		if (!dataHandlerJson.equals("default") && usingLaptop) {
			try {
				// Parse JSON string
				Type mapType = new TypeToken<Map<String, String>>() {}.getType();
				Map<String, String> dataFromPython = gson.fromJson(dataHandlerJson,
						mapType);
				if (dataFromPython.containsKey("modelUpdated")) {
					if (responseData.containsKey("shouldUpdateModel")) {
						responseData.remove("shouldUpdateModel"); //stop asking for data
					}
				}
				// Prepare response data
				responseData.put("status", "running");
				// Convert response data to JSON
				String jsonResponse = gson.toJson(responseData);
				// Send response JSON to Python
				SmartDashboard.putString("FromRobot", jsonResponse);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		} //Laptop not being used.
		try (Socket socket = serverSocket.accept();
				PrintWriter out = new PrintWriter(socket.getOutputStream(), true);
				BufferedReader in = new BufferedReader(
						new InputStreamReader(socket.getInputStream()))) {
			// Receive JSON string from Orange Pi
			StringBuilder jsonStringBuilder = new StringBuilder();
			/*char[] buffer = new char[1024];
			int charsRead;
			// Read data in chunks of 1024 characters
			while ((charsRead = in.read(buffer)) != -1) {
				jsonStringBuilder.append(buffer, 0, charsRead);
				// Optional: Break if the message has a specific delimiter, e.g., newline
				if (jsonStringBuilder.toString().contains("\n")) {
					break;
				}
			}
			String jsonString = jsonStringBuilder.toString().trim();
			*/
			int character;
			while ((character = in.read()) != -1) {
				char c = (char) character;
				if (c == '\n') {
					break; // Reached delimiter, stop reading
				}
				jsonStringBuilder.append(c);
			}
			String jsonString = jsonStringBuilder.toString();
			// Parse JSON string
			JsonObject receivedData = JsonParser.parseString(jsonString)
					.getAsJsonObject();
			Logger.recordOutput("OrangePi/rawFromPi", jsonString);
			// Process received data (optional)
			if (receivedData.has("timestamp")) {
				int time = receivedData.get("timestamp").getAsInt();
				if (time != oldTime + 1) {
					SmartDashboard.putString("PiConnection",
							"SKIPPED VIA SOCKET" + oldTime);
				}
				oldTime = time;
				//System.out.println("time: " + time);
			}
			if (receivedData.has("outputs")) {
				if (responseData.containsKey("modelInputs")) {
					responseData.remove("modelInputs");
				}
				String rawData = receivedData.get("outputs").getAsString();
				List<Double> list = makeDoubleList(rawData); //index 0 = velocity of topShooter, index 1 = velocity of bottomShooter, index 2 = angle of shooter
				list.add(0, TimeUtil.getLogTimeSeconds()); //those above shifted 1
				///RobotContainer.currentAiOutputs = list;
				//Remove brackets
			}
			// Interaction with Double Jointed Arm
			if (receivedData.has("voltages")) {
				String rawData = receivedData.get("voltages").getAsString();
				@SuppressWarnings("unused")
				List<Double> voltages = makeDoubleList(rawData);
				long currentTime = Logger.getTimestamp();
				double latency = (currentTime - oldTimestamp)/1e6;
				Logger.recordOutput("DoubleJointedArmS/Latency", latency);
				oldTimestamp = currentTime;
			
				//System.out.println(voltages);
			}
			if (receivedData.has("gotEncoder")){
				if (responseData.containsKey("DoubleJointedEncoders")){
					responseData.remove("DoubleJointedEncoders");
				}
			}
			if (receivedData.has("gotConstants")){
				if (responseData.containsKey("DoubleJointedArmConstants")){
					responseData.remove("DoubleJointedArmConstants");
				}
			}
			responseData.put("status", "running");
			// Prepare response JSON
			if (receivedData.has("currentStatus")) {
				Logger.recordOutput("OrangePi/PiConsole",
						receivedData.get("currentStatus").toString());
			} else {
				Logger.recordOutput("OrangePi/PiConsole",
						"UNKNOWN ERROR - RIO OVERRIDED THIS ERROR");
			}
			String jsonResponse = gson.toJson(responseData);
			Logger.recordOutput("OrangePi/ToPiJson", jsonResponse);
			// Convert JSON to string and send as response
			// Send the JSON response in 1024-byte chunks
			out.println(jsonResponse);  // Send response to client
			RobotContainer.piConnection = "OK";
			Logger.recordOutput("OrangePi/PiConnection", "OK");
		}
		catch (Exception e) {
			//e.printStackTrace();
			RobotContainer.piConnection = "DISCONNECTED";
			Logger.recordOutput("OrangePi/PiConnection", "PI NOT DETECTED");
		}
	}
}
