// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Touchboard;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.SubsystemBase;

public class ToggleButton extends SubsystemBase {

  final BooleanSubscriber dataSubscriber;
  final BooleanPublisher dataPublisher;

  String buttonName;
  Command executed;

  public ToggleButton(String buttonName, Command executed) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("touchboard");

    dataPublisher = datatable.getBooleanTopic(buttonName).publish();
    dataSubscriber = datatable.getBooleanTopic(buttonName).subscribe(false);

    this.executed=executed;
  }
  public boolean getValue() {
    return dataSubscriber.get();
  } 

  public void periodic() {
    boolean value = dataSubscriber.get();

    if (value) {
      CommandScheduler.getInstance().schedule(executed);
    }else{
      executed.cancel();
    }
  }

  public void close() {
    dataSubscriber.close();
  }
}
