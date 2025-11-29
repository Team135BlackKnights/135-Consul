// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Touchboard;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
