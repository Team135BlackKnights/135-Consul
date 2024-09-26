#!/bin/bash

# Path to the Constants.java file
CONSTANTS_FILE="src/main/java/frc/robot/Constants.java"

# Use sed to find and replace the currentMode assignment
sed -i 's/currentMode = .*;/currentMode = Mode.SIM;/' "$CONSTANTS_FILE"

echo "Updated currentMode to Mode.SIM in $CONSTANTS_FILE"

# Run the FRC simulator in the background
./gradlew simulateJava &

# Capture the PID of the simulator
SIMULATOR_PID=$!

# Allow the simulator to run for 30 seconds
sleep 30

# Stop the simulator
kill $SIMULATOR_PID