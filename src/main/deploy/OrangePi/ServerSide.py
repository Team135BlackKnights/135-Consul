import os
import socket
import sys
import threading
import json
import time
import re
import io
import joblib
import numpy as np
from frccontrol import DcBrushedMotor
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor

#Custom Imports
from DoubleJointedArm import DoubleJointedArm

#Gradient Booster is a tree algorithm that uses "trees" (estimators) like neural network layers.
model = GradientBoostingRegressor(n_estimators=100)
#We are always going to be using at least one input and one output, so we require this wrapper
wrapper = MultiOutputRegressor(model)

#Do not edit the directories. It will crash.
model_naming = re.compile(r"model_v(\d+)\.pkl")
directory = "../ModelsPI/"
MOTOR_KRAKEN_X60_FOC = DcBrushedMotor(12.0, 9.36, 476.1, 2, 6000.0)
dt = 0.02  #what's the periodic time OF DoubleJointedArmS.java ?
length1 = 46.25 * .0254  # in meters, so .0254 conversion factor
length2 = 41.8 * .0254
# Mass of segments
mass1 = 9.34 * 0.4536  #In KG
mass2 = 9.77 * 0.453

# Distance from pivot to CG for each segment
pivot_to_CG1 = 21.64 * 0.0254  #in meters
pivot_to_CG2 = 26.70 * 0.0254

# Moment of inertia about CG for each segment
MOI1 = 2957.05 * 0.0254 * 0.0254 * 0.4536  # In KgM^2
MOI2 = 2824.70 * 0.0254 * 0.0254 * 0.4536

# Gearing of each segment
gearing1 = 70.0  #Reduction is greater than 1.
gearing2 = 45.0

# Number of motors in each section
motor_count1 = 1  #How many motors on arm
motor_count2 = 1  #How many motors on elbow

# Motor Type. ALL ARM MOTORS MUST BE SAME
motor_type = MOTOR_KRAKEN_X60_FOC
# Gravity
gravity = 9.806
# Qelms/Relms (ONLY TWEAK THIS!
q_pos = 0.01745
q_vel = 0.1745329
q_error = 10
r_pos = 0.01745 / 4
#Real controls and outputs.
voltages = np.zeros(4)
angleShoulder = 0  #auto updated on boot
angleElbow = 0  #auto updated on boot
stop_event = threading.Event()

def to_state(x, y, invert, arm):
    """
    :param arm: what arm to make the state for
    :param x: in meters for left/right
    :param y: in meters for top/bottom
    :param invert: should the elbow be left or right of the setpoint?
    :return: numpy array with arm theta, elbow theta, and two velocities at the setpoint (0)
    """
    theta1, theta2 = arm.constants.inv_kinematics(x, y, invert)
    return np.array([[theta1], [theta2], [0], [0]])


def initialize_arm(target_state):
    """
    Initialize the arm.
    """
    return DoubleJointedArm(dt, length1, length2, mass1, mass2, pivot_to_CG1, pivot_to_CG2, MOI1, MOI2, gearing1,
                            gearing2, motor_count1, motor_count2, motor_type, gravity, target_state, q_pos, q_vel,
                            q_error, r_pos)


def initialize_arm_with_encoders():
    """
    Initialize the arm's state based on encoder readings. does NOT accept given velocity, as latency compensation
    already consumes those values.
    """
    target_state = np.array([[angleShoulder], [angleElbow], [0],
                             [0], [0], [0]])
    arm = initialize_arm(target_state)
    # Set initial state based on encoder values
    arm.x = target_state  # Initial state [angle1, angle2, velocity1, velocity2]
    arm.observer.x_hat = arm.x  #override the Kalman filter to have the target_state (since it's our start pos)

    return arm


def arm_loop(arm):
    """
    Run the arm. Usually done in a separate thread.
    :param arm: arm to run
    :return: never, run in a thread.
    """
    #Uncomment the commented lines, and COMMENT the ones with COMMENT FOR MATPLOT for a visual output ON the pi
    #(almost never done)
    global voltages
    #fig, ax, arm_line, ref_line = initialize_plot_live(arm)
    while not stop_event.is_set():
        arm.update()
        #arm_line, ref_line = update_plot_live(arm, arm_line, arm.target_state, ref_line)
        volts = arm.getVolt()
        positions = arm.getPositions()
        voltages = np.concatenate((volts, positions), axis=0)
        time.sleep(arm.dt)  #COMMENT FOR MATPLOT
        #plt.draw()
        #plt.pause(arm.dt)


arm = None
arm_thread = None
failCount = 0


def update_arm_tunables(q_pos_new, q_vel_new, q_error_new, r_pos_new):
    global q_pos, q_vel, q_error, r_pos, arm, arm_thread
    q_pos = q_pos_new
    q_vel = q_vel_new
    q_error = q_error_new
    r_pos = r_pos_new
    if arm_thread is not None:
        stop_event.set()  # Signal the thread to stop
        arm_thread.join()  # Wait for the thread to finish
        arm_thread = None
        stop_event.clear()  # Reset the event for the next thread

        # Reset the arm object
    arm = None


def send_to_roborio(data, roborio_ip, roborio_port):
    """
    Send a given JSON to the roboRIO, and receive the response from the roboRIO.
    Any specific value from the response JSON will be set up here.
    :param data: JSON being sent to the RIO.
    :param roborio_ip: depending on if sim or not (10.1.35.2 or localhost)
    :param roborio_port: almost always 5802.
    """
    global angleShoulder, angleElbow, arm, failCount, nextConsoleLog, arm_thread
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((roborio_ip, roborio_port))
        #add the nextConsoleLog to data
        data['currentStatus'] = nextConsoleLog
        s.sendall((json.dumps(data) + '\n').encode())  #send the data
        nextConsoleLog = []
        try:
            received_data = b''
            while True:
                chunk = s.recv(1024)
                if not chunk:
                    break  # No more data from the server
                received_data += chunk  # Append each chunk to the full message

                # Optionally: Check if a known delimiter (like newline '\n') exists in chunk
                if b'\n' in received_data:
                    break  # We assume the message is complete when we see a newline character

            # Decode the compiled data (after the loop ends)
            json_string = received_data.decode('utf-8').strip()  # Remove any extra newlines/whitespace
            print("Received raw data:", json_string)

            # Parse the JSON string
            data_from_robot = json.loads(json_string)
            print("Received JSON:", data_from_robot)
            #If you're using a confirmed send (response contains a marker) you'll need an else on your if check.
            if "DoubleJointedEncoders" in data_from_robot:
                encoders_string = data_from_robot["DoubleJointedEncoders"]
                encoders = [float(value) for value in encoders_string.split(",")]
                angleShoulder, angleElbow = encoders[0], encoders[1]
                #If we haven't yet created the arm, do it here. This allows for the OrangePi to have zero needed pushes
                #so that we avoid having to deal with uploading code to the Pi's. All done via ClientSide (hopefully)
                if arm is None:
                    arm = initialize_arm_with_encoders()
                    stop_event.clear()  # Reset stop_event for the new thread
                    arm_thread = threading.Thread(target=arm_loop, args=(arm,))
                    arm_thread.daemon = True  # Allow the thread to be terminated when the main thread exits
                    arm_thread.start()
                arm.updatePosition(angleShoulder, angleElbow)
                data["gotEncoder"] = str("RECEIVED ENCODERS")
            else:
                if 'gotEncoder' in data:
                    data.pop('gotEncoder')
            if "DoubleJointedArmConstants" in data_from_robot:
                constants_string = data_from_robot["DoubleJointedArmConstants"]
                constants = [float(value) for value in constants_string.split(",")]
                update_arm_tunables(constants[0], constants[1], constants[2], constants[3])
                data["gotConstants"] = str("RECEIVED CONSTANTS")
                print("UPDATED!")
            else:
                if 'gotConstants' in data:
                    data.pop('gotConstants')
            if "modelInputs" in data_from_robot:
                m_input_string = data_from_robot["modelInputs"]
                m_input = [float(value) for value in m_input_string.split(",")]
                np_input = np.array(m_input)
                outputs = runValue(np_input)
                print(outputs)
                data["outputs"] = str(outputs)
            else:
                if 'outputs' in data:
                    data.pop('outputs')
            if "DoubleJointSetpoint" in data_from_robot:
                rawData = data_from_robot["DoubleJointSetpoint"]
                wantedPos = [float(value) for value in rawData.split(",")]
                if arm is None:
                    arm = initialize_arm_with_encoders()
                    stop_event.clear()  # Reset stop_event for the new thread
                    arm_thread = threading.Thread(target=arm_loop, args=(arm,))
                    arm_thread.daemon = True  # Allow the thread to be terminated when the main thread exits
                    arm_thread.start()
                newState = to_state(wantedPos[0], wantedPos[1], bool(wantedPos[2]), arm)
                arm.set_target_state(newState)
            failCount = 0
        except Exception as e:  # we use a global exception case to prevent crashing here, it gives ugly behaviour.
            failCount += 1
            print("FAILED TO READ BECAUSE OF EXCEPTION")
            print(str(e))
            pass  # don't crash.


def latest_model():
    """
    Check the directory for models, and get the most recent one.
    :return: latest model like "model_v2.pkl"
    """
    versions = []

    # Create the directory if it doesn't exist
    os.makedirs(directory, exist_ok=True)
    for filename in os.listdir(directory):
        match = model_naming.match(filename)
        if match:
            versions.append(int(match.group(1)))
    if versions:
        latest_version = max(versions)
        return f'model_v{latest_version}.pkl'
    else:
        return None


def load_latest_model():
    """
    Load the most recent model.
    :return: nothing, but update our model wrapper
    """
    global wrapper
    # Get list of subdirectories in the Models directory
    newest_model = latest_model()

    if not newest_model:
        print("No models found. Do not use runValue.")
    # Sort directories by creation time (modification time of the directory)
    else:
        newest_model_path = os.path.join(directory, newest_model)
        print("Using " + newest_model)
        wrapper = joblib.load(newest_model_path)


def handle_client(conn, addr):
    """
    Handle client connection to PyDriverStation, automatically downloading the model sent from PyDriverStation
    :param conn: what data is being sent
    :param addr: whatever IP and port the client is on (varies from call to call)
    :return: the new model, and save to ../OrangePi/Models/Model_v{index}.pkl
    """
    global wrapper
    print('Connected by', addr)
    data = b''
    while True:
        packet = conn.recv(1024)
        if not packet:
            break
        data += packet
    newest_model = latest_model()
    if newest_model:
        print("Current Latest Model is: " + newest_model)
        latest_version = int(model_naming.match(newest_model).group(1))
        new_version = latest_version + 1
    else:
        new_version = 1
    new_model = f'model_v{new_version}.pkl'
    new_model_path = os.path.join("../OrangePi/ModelsPI/", new_model)
    print(new_model_path)
    # Write data to a file in the specified directory
    with open(new_model_path, 'wb') as temp_file:
        temp_file.write(data)

    # Load the wrapper.pkl file using pickle
    with open(new_model_path, 'rb') as file:
        wrapper = joblib.load(file)


def runValue(value):
    """
    Calculate model output with given value
    :param value: numpy array with inputs, like [4.5,2.4,6]
    :return: numpy array with outputs
    """
    try:
        yhat = wrapper.predict([value])
        return yhat[0]
    except Exception as e:
        print(e)
        return 0
    # summarize the prediction
    # print('Predicted: %s' % yhat[0])


# Buffer to store console output
nextConsoleLog = []


# Custom class to capture stdout and stderr
# Custom print function to capture both stdout and stderr
def custom_print(*args, **kwargs):
    # Convert all args to a single string
    output = ' '.join(map(str, args))
    # Capture the log
    nextConsoleLog.append(output)
    # Call the original print function to print to console
    sys.__stdout__.write(output + "\n")  # Use sys.__stdout__ to avoid recursion with custom print


# Override the built-in print function with our custom function
print = custom_print


def main():
    HOST = '0.0.0.0'  # Listen on all available interfaces
    PORT = 5801  # Port to listen on for PyDriverStation
    data_to_robot = {'timestamp': '0'}
    roborio_ip = '10.1.35.2'  # Replace with the actual IP address of the roboRIO (10.1.35.2)
    roborio_port = 5802  # Port on which the roboRIO is listening
    heartbeat = 0
    load_latest_model()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("BINDING...")
        s.bind((HOST, PORT))
        s.listen()
        s.settimeout(.005)
        print(f'Server listening on {HOST}:{PORT}')
        while True:
            heartbeat += 1
            #read the console, and set nextConsoleLog to it

            try:
                conn, addr = s.accept()
                client_thread = threading.Thread(target=handle_client, args=(conn, addr))
                client_thread.start()
            except TimeoutError:
                pass
            except Exception:
                s.close()
                print("CRASHED BECAUSE CONNECTION TERMINATED... REBOOTING")
                raise ConnectionError
            data_to_robot['voltages'] = str(voltages)  # 3/4 are the wanted positions #5/6 are the (expected) velocities
            data_to_robot['timestamp'] = str(heartbeat)
            try:
                send_to_roborio(data_to_robot, roborio_ip, roborio_port)
            except TimeoutError:
                print("Timed Out RIO")
                pass
            except Exception:
                s.close()
                print("CRASHED BECAUSE CONNECTION TERMINATED... REBOOTING")
                raise ConnectionError
            if failCount > 5:
                print("Restarting due to too many failed reads in a row.")
                raise ConnectionError
            #data_to_robot.clear()

            # Send the received data to the roboRIO


# BEFORE YOU CALL MAIN MAKE SURE: (only use localhost if sim)
# 1. HOST (laptop) = 10.1.35.5  OR localhost
# 2. roborio_ip = 10.1.35.2 OR localhost
# 3. You have NOT touched send_to_roborio.
# 4. If you're using the Double Jointed Arm, be SURE to go into DoubleJointedArm.py and tweak "r_pos" DOWN until it
# breaks the REAL ARM, NOT SIM.
# Moving any lines in this function will CRASH THE PI (robot will keep working), I almost promise you.
main()
