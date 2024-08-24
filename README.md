![blocks_logo](https://github.com/Team135BlackKnights/135-Blocks/assets/49589065/488cddd6-688f-4a2d-b0a6-cc395535b318)

# 135 Blocks
This is designed to be a template-based framework for FRC programming.

This is designed so that the only files that need changing to combine different subsystems is RobotContainer.java if used properly.

### Built-in Features for Each Block
- Drivetrain support
  - Swerve
  - Mechanum
  - Tank
- Multi-Vendor support
  - CTRE Motors
  - REV Spark Maxes
  - REV Spark Flexes
- Custom API for interfacing with Custom AI on Orange Pi 5
  - Allows any number of inputs
  - Allows any number of outputs
  - Prevents Overfitting
  - Displays all useful statistics for human-verification
  - Essentially ZERO latency (.0003 seconds)
  - Zero knowledge of AI required
- Motor Constant Container
  - Prevent mistakes/crash-causing errors from SysID via checks
  - Put all SysID in ONE place

![Simulation with vision and physics](Documentation/physics_plus_vision.gif)
- FULL Sim Support
- Swerve physics, including field collision
- Dynamic PID Tuning 
  - Loggable Tuning Numbers allow updates
- SysId Selector
  - Use a testing controller to select SysId tests on different subsystems
  - Automatically appends all tests needed for the new block to the selector
- Drive to Pose Command
  - Slow/Fast mode, where each has custom max accel/velocity
  - Go to a given Pose2d
  - Pathfinds on its own using ADStar planning
- Aim to Pose Command
  - Aim at a given Pose2d
  - Works IN PARALLEL with both TeleOp drive/Pathplanner drive  
- Orchestra
  - Orchestra for CTRE
  - Plays a given `.chrp` file to ALL connected TalonFXs

![Branching Auto](Documentation/branching_auto_no_physics.gif)
- Branching Autos
  - Automatically updates the auto based off game state, and what happened in the last path
  - Has to FOLLOW a race command group, which has the prior auto path and an interrupt command based off game status
  - Will run the given Choreo trajectory UNLESS the GamePiece status is not zero
  - Will pathfind to a given Pose2d if gamepiece status is not zero



### SubsystemChecker Overhaul: 
The base branch includes an overhauled Subsystem system named `SubsystemChecker`. It provides system checking and outputs data to our Pit Display, where all of this data is visible. All Subsystems should extend `SubsystemChecker` and supply their own TestCommand to confirm ALL functionality of that subsystem, that way the Pit Display can run everything on its own. 
All SubsystemChecker extensions include:
- Self-Checking hardware, which throws errors to the Pit Display/Advantage Scope if any faults occur ANYWHERE on the device.
  - Talon FX
  - CAN Spark Base
  - Pigeon 2 / NavX 2
  - CANCoder
  - PWMMotor
- Easy to use SystemCheckCommand, which will run THAT subsystem's check, confirming functionally by throwing faults if speeds/positions are off.
- Built-in Orchestra, allowing easy Orchestra use.

### Directory Structure
Preferably, do NOT touch any of the `drive/*` contents, as that could cause a merge error!
##### However, some files do need to change interactions with the base, so it is understandable in some situations.

An example branch directory could be:

```
    src/main/java/frc/robot/
        commands/
        drive/
            ...
        exampleBranchName/
            exampleCommand.java
        subsystems/
        drive/
            ...
        exampleBranchName/
            exampleSubsystem.java
        utils/
        drive/
            ...
        exampleBranchName/
            Constants.java
        Constants.java
    DataHandler.java
    Main.java
    Robot.java
    RobotContainer.java
```

## Usage

Refer to the [PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation) repository for instructions on how to use the API for the neural network.
Refer to the [135_Pit_Display](https://github.com/Team135BlackKnights/135_Pit_Display) repository for instructions on how to use the Pit Display testing and diagnostics.


## Credits

We'd like to extend our deepest gratitude to FRC teams 254, 449, 1678, 1690, 3015, 5516, 6328 and so many others for their code and ideas. Links to their codebases are here:  
[254  (The Cheesy Poofs)](https://github.com/Team254): Setpoint Generator (accel limiting)  
[1678 (Citrus Circuits)](https://github.com/frc1678): Idea for vision filtering  
[1690 (Orbit)](https://github.com/team1690): Skid Detection Algorithm  
[3015 (Ranger Robotics)](https://github.com/3015rangerrobotics): Pathplanner, Pit Display (event panel)  
[5516 (Shenzhen Robotics Alliance)](https://github.com/Shenzhen-Robotics-Alliance): Simulation physics logic  
[6328 (Mechanical Advantage)](https://github.com/Mechanical-Advantage): Advantage Kit, Advantage Scope, 250 Hz Advanced Skeleton Swerve, URCL, DriveToPose  
[PhotonVision](https://photonvision.org): Photon Vision
[Dyn4j](https://github.com/dyn4j/dyn4j): Base physics engine for simulation  
[ChatGPT](https://chat.openai.com/): For helping us feel not alone

## Blocks

### State Space

This block contains:
- Pre-made configurable mechanisms:
  - Double Jointed Arm (For CTRE Only)
  - Elevator (cascade lifts, telescoping arms)
  - Flywheel
  - Single Jointed Arm

#### Commands
- `CTREDoubleJointedArmC`: Update arm macro position setpoints, given a wanted elbow point direction, and grab voltages from the best possible path to that position using the OrangePi. Hold at end setpoint.
- `CTREElevatorC`: Move the elevator to a desired macro position, or move it via a joystick, and hold there.
- `CTREFlywheelC`: Accelerate the wheels to a desired RPM, or using a joystick, *returning to zero when let go*.
- `CTRESingleJointedArmC`: Move the arm to a desired macro position, or move it via a joystick, and hold there.
- `REVArmC`: Move the arm to a desired macro position, or move it via a joystick, and hold there.
- `REVElevatorC`: Move the elevator to a desired macro position, or move it via a joystick, and hold there.
- `REVFlywheelC`: Accelerate the wheels to a desired RPM, or using a joystick, *returning to zero when let go*.

#### Tests
- `CTREDoubleJointedArm`: Goes to a given macro, waits 5 seconds, and confirms it arrived at the position. Then go to a second macro, and repeat. Ends after second macro is confirmed. Error 5 inches for both x/y.
- `CTREElevator`: Moves the elevator up to 2 ft, waits 2 seconds, and confirms it arrived at the position. Then go back to zero, and repeat. Ends after 0 ft is confirmed. Error 4 in
- `CTREFlywheel`: Sets to 4000 RPM, waits 1.5 seconds, and confirms it arrived at the speed. Then go up to 6000, and repeat. Ends after 6000 is confirmed, setting to zero after. Error 50 RPM.
- `CTRESingleJointedArm`: Moves the arm to 45 degrees, waits two seconds, and confirms it arrived at the position. Error 5 degrees.
- `REVElevator`: Moves the elevator up to 2 ft, waits 2 seconds, and confirms it arrived at the position. Then go back to zero, and repeat. Ends after 0 ft is confirmed. Error 4 in
- `REVFlywheel`: Sets to 4000 RPM, waits 1.5 seconds, and confirms it arrived at the speed. Then go up to 6000, and repeat. Ends after 6000 is confirmed, setting to zero after. Error 50 RPM.
- `REVSingleJointedArm`: Moves the arm to 45 degrees, waits two seconds, and confirms it arrived at the position. Error 5 degrees.


#### Simulation
All mechanisms work in simulation and **require** SysId constants. For the Double Jointed Arm, the Orange Pi Server MUST be running. (Sim version supported) 

#### Notes
For more details on System Identification, refer to the [WPILib System Identification documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html).
For more details on State Space, refer to the [WPILib State Space Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html).
For more details on Orange Pi Server, refer to the [OrangePi code in PyDriverStation](https://github.com/Team135BlackKnights/PyDriverStation/tree/main/OrangePi).

### Cameras, Photon-Vision, and Limelight
![Vision Wireframe view from Photon Vision](Documentation/photonvision_wireframe.gif)

This block contains:
- 4 built-in bot-pose cameras.
- Rigorous confirmation of positioning before updating the robot position for maximum accuracy.
- Built-in AprilTag Trust system: 
  - Decreases trust if a tag provides incorrect estimates.
  - Increases trust back to 100% if a tag improves over time.
- Accurate Limelight Sim for tX, tY, and tV.
  - Using a limelight in Sim is 100% accurate, for better or for worse.
  - Can be used to tune Vision PIDs, as it uses the SAME tX and tY calculations.

#### Commands
- `DriveToAITarget`: Use pipeline zero on the limelight, and aim towards it and accelerate towards it until a desired distance is achieved. Maps speeds so that it doesn't stop suddenly at the end.
- `AimToApriltag`: Uses a given ID to automatically update the robot rotation to aim towards that tag, and works during pathplanner. Uses ambiguity checks and avoids constant oscillations.

#### Tests
- Checks to make sure all cameras are connected, and has no command. Will be green unless a camera disconnected.
#### Simulation
Works entirely in simulation. Access all PhotonVision cameras via your browser. Limelight is supported in simulation mode, however it cannot be visualized via a wireframe.

- `localhost:1190/stream.mjpg`
- `localhost:1192/stream.mjpg`
- `localhost:1194/stream.mjpg`
- `localhost:1196/stream.mjpg`

Add these to Shuffleboard for ease of access!

### LED_Code

This block contains:
- Pre-made addressable LEDs with simulation support.
- Pre-made commands for:
  - Using videos from a USB stick on the RIO.
  - Displaying patterns like rainbows, breathing effects, and constant colors.

#### Commands
- `LEDGifC`: Efficiently displays any image in a specified folder on the USB stick, at a given interval. Loops after the last photo.
- `LEDSineWaveC`: Displays a sine wave that moves through all the LEDs, at a given interval and color.
- `LEDBreathingC`: Breathes a certain color, gradually increasing and decreasing brightness, at a given interval.
- `LEDConstantColorC`: Displays a constant color.
- `LEDRainbowC`: Cycles through the rainbow at a given interval.

#### Tests
- Runs a debug image for 5 seconds, then ends. User MUST check to make sure the Debug image was okay, it does NOT self-check that.

#### Notes
The system saves images on BOOT, ***not deploy***, so that if the USB is unplugged during the match, no problems occur. It does this to save space on the RIO itself, as there isn't much storage (16mb), but there is a lot of RAM (512 mb).
In order to use the `LEDGifC` command, you must first split up the GIF into its component frame images and upload those component images into the /util/images folder, as well as the USB stick /images folder.

#### Simulation
All LED patterns are supported in simulation.

### Servos

This block contains:
- Custom servo sim that allows you to set the bounds, and simulate a REVSmartServo’s continuous mode
- Custom enum that dictates servo type as well as which mode the servo is being run in
- Custom servoPackage util that allows for seamless transitions between real robot and simulation

#### Commands
- `ServoC`: Sets the servos via `setServoDegrees`. Automatically handles Sim.

#### Tests
- Sets left servo to 45 deg, right to -60, waits 1 second, and confirms both arrived at their positions. Then both go to zero, and repeat. Ends after both are confirmed back at zero. Error 10 degrees.
#### Simulation
All Servos are supported in simulation.

#### Notes
To add more servos, create a new `servoPackage` object and add it to the array of existing ones located in `servoC`. In addition to that, create a new enumeration in the `ServoNames` enum, and update the case statements to include the new `servoPackage`. 

### Solenoid-Code

This block contains:
- Pre-made single-acting and double-acting solenoids.
- A (questionable) function to hold pneumatic cylinders at certain positions using a bang-bang PID-like controller.

#### Notes
NO sim/test support. It's a solenoid!

### Wrappers

All blocks need two wrappers to be used in order to function:
- `MotorConstantContainer`: A wrapper that holds characterization values (ks, kv, ka, kp, kd) of a particular motor. Throws an error if an incorrect value is input. Used for state-space models and drivetrain simulation.
- `IO`: A wrapper that allows for efficient communication with component. Needs to be written independently for each component. Check the `utils` folders for example implementations.

## MatchState 

 This value lets you see what part of the FRC Match the robot is in, including endgame/test. Works during practice matches.

## Getting Started
 In order to get started with this system, first make a fork of this repository. After that is done, merge the branches with the features that you need into the main branch, and tweak constants based on your robot. If you want to use simulation, make sure you are familiar with AdvantageKit, AdvantageScope and the PyDriverStation as this repository relies on all three to function properly.

## Workflow
 This repository is designed so that multiple programmers can merge their work from separate branches without any issues. In order to accomplish this, create a folder with the name of each branch under all of the `utils` folders.  If this is done correctly, the only merge issues that  
 need to be resolved should be in RobotContainer.java.
