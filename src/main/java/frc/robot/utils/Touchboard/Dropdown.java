// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Touchboard;
import org.wpilib.command2.SubsystemBase;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

import java.util.function.Supplier;
import org.wpilib.command2.Commands;

public class Dropdown extends SubsystemBase {
  public String value = "";
  String prev = "";

  String topic;

  StringSubscriber dataSubscriber;
  StringPublisher dataPublisher;

  Supplier<Command> passedCommand = () -> Commands.none();

  public Dropdown(String topic) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("touchboard");

    dataPublisher = datatable.getStringTopic(topic).publish();
    dataSubscriber = datatable.getStringTopic(topic).subscribe("");
    
    this.topic = topic;
  }

  public void setCommand(Supplier<Command> newCom) {
    passedCommand = newCom;
  }

  public String getValue() {
    return dataSubscriber.get();
  }

  @Override
  public void periodic() {
    value = dataSubscriber.get();

    if (!value.equals(prev)) {
      prev = value;
      CommandScheduler.getInstance().schedule(passedCommand.get());
    }
    // This method will be called once per scheduler run
  }
}
