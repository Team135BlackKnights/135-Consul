// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Touchboard;
import org.wpilib.command2.SubsystemBase;

import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

import java.util.function.Supplier;
import org.wpilib.command2.Commands;

public class AxisKnob extends SubsystemBase {
  /** Creates a new AxisKnob. */
  public double value = 0;
  double prev = 0;
  String topic;
  
  DoubleSubscriber dataSubscriber;
  DoublePublisher dataPublisher;

  Supplier<Command> passedCommand = () -> Commands.none();

  public AxisKnob(String topic) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("touchboard");

    dataPublisher = datatable.getDoubleTopic(topic).publish();
    dataSubscriber = datatable.getDoubleTopic(topic).subscribe(0);
    
    this.topic = topic;
  }

  public void setCommand(Supplier<Command> newCom) {
    passedCommand = newCom;
  }

  public double getValue() {
    return dataSubscriber.get();
  }

  @Override
  public void periodic() {
    value = dataSubscriber.get();

    if (value != prev) {
      prev = value;
      CommandScheduler.getInstance().schedule(passedCommand.get());

    }
    // This method will be called once per scheduler run
  }
}
