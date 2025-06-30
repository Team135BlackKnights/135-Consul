package frc.robot.utils.operatorDashboard;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Paths;

public class OperatorDashboardIOServer implements OperatorDashboardIO {
  private static final String toRobotTable = "/OperatorControls/ToRobot";
  private static final String toDashboardTable = "/OperatorControls/ToDashboard";
  private static final String topRowTopic = "TopRow";
  private static final String midRowTopic = "MidRow";
  private static final String hybridRowTopic = "HybridRow";
  private static final String targetBoxTopic = "TargetBox";

  private final IntegerSubscriber topRowIn;
  private final IntegerSubscriber midRowIn;
  private final IntegerSubscriber hybridRowIn;
  private final StringSubscriber targetBoxIn;

  private final IntegerPublisher topRowOut;
  private final IntegerPublisher midRowOut;
  private final IntegerPublisher hybridRowOut;
  private final StringPublisher targetBoxOut;


  public OperatorDashboardIOServer() {
    var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);
 
    topRowIn = inputTable.getIntegerTopic(topRowTopic)
        .subscribe(0, PubSubOption.keepDuplicates(true));
    midRowIn = inputTable.getIntegerTopic(midRowTopic)
        .subscribe(0, PubSubOption.keepDuplicates(true));
    hybridRowIn = inputTable.getIntegerTopic(hybridRowTopic)
        .subscribe(0, PubSubOption.keepDuplicates(true));
    targetBoxIn = inputTable.getStringTopic(targetBoxTopic) 
        .subscribe("", PubSubOption.keepDuplicates(true));

    var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);
    topRowOut = outputTable.getIntegerTopic(topRowTopic).publish();
    midRowOut = outputTable.getIntegerTopic(midRowTopic).publish();
    hybridRowOut = outputTable.getIntegerTopic(hybridRowTopic).publish();
    targetBoxOut = outputTable.getStringTopic(targetBoxTopic).publish();

    WebServer.start(
        5801,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "OperatorDashboard").toString()
    );
  }

  @Override
  public void updateInputs(OperatorDashboardIOInputs inputs) {
    inputs.topRow = topRowIn.readQueue().length > 0
        ? new int[] { (int) topRowIn.get() }
        : new int[] {};

    inputs.midRow = midRowIn.readQueue().length > 0
        ? new int[] { (int) midRowIn.get() }
        : new int[] {};

    inputs.hybridRow = hybridRowIn.readQueue().length > 0
        ? new int[] { (int) hybridRowIn.get() }
        : new int[] {};
    inputs.targetBox = targetBoxIn.readQueue().length > 0
        ? targetBoxIn.get()
        : "";
  }

  @Override
  public void setTopRow(int value) {
    topRowOut.set(value);
  }

  @Override
  public void setMidRow(int value) {
    midRowOut.set(value);
  }

  @Override
  public void setHybridRow(int value) {
    hybridRowOut.set(value);
  }
  @Override
  public void setTargetBox(String val) {
    targetBoxOut.set(val);
  }
}
