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

public class DoubleActionButton extends SubsystemBase {

  final BooleanSubscriber dataSubscriber;
  final BooleanPublisher dataPublisher;
  
  String buttonName;
  boolean prev = false;

  Command executed;
  Command Finished;
  
  public DoubleActionButton(String buttonName, Command executed, Command Finished) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("touchboard");

    dataPublisher = datatable.getBooleanTopic(buttonName).publish();
    dataSubscriber = datatable.getBooleanTopic(buttonName).subscribe(false);
    
    this.Finished = Finished;
    this.executed = executed;
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

    if(!value && prev != value){
      CommandScheduler.getInstance().schedule(Finished);
    }
    prev = value;
  }

  public void close() {
    dataSubscriber.close();
  }
}
