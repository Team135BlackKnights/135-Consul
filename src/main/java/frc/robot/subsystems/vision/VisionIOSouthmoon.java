package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.vision.VisionConstants;

/**
 * Southmoon implementation of VisionIO.
 * Uses the Southmoon AprilTag and object detection system.
 */
public class VisionIOSouthmoon implements VisionIO {
  private final Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier;
  private VisionConstants.AprilTagLayoutType lastAprilTagLayout = null;

  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber objDetectObservationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final IntegerSubscriber fpsObjDetectSubscriber;
  private final StringPublisher eventNamePublisher;
  private final IntegerPublisher matchTypePublisher;
  private final IntegerPublisher matchNumberPublisher;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;
  private final StringPublisher tagLayoutPublisher;
  private final FloatArrayPublisher fieldCameraPosePublisher;
  private final int camIndex; 
  private final Timer slowPeriodicTimer = new Timer();

  /**
   * Creates a new VisionIOSouthmoon.
   * 
   * @param aprilTagLayoutSupplier Supplier for the current AprilTag layout
   * @param index Camera index for this Northstar instance
   * @param cameraConfig Camera configuration (from your VisionConstants)
   */
  public VisionIOSouthmoon(
      Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier, 
      String id,
      int i,
      VisionConstants.CameraConfig cameraConfig) {
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
    this.deviceId = id;
    
    var northstarTable = NetworkTableInstance.getDefault().getTable(this.deviceId);
    var configTable = northstarTable.getSubTable("config");

    // Publish camera configuration
    configTable.getStringTopic("camera_id").publish().set(cameraConfig.getId());
    configTable.getStringTopic("camera_location").publish().set(cameraConfig.getLocation());
    configTable.getIntegerTopic("camera_resolution_width").publish().set(cameraConfig.getWidth());
    configTable.getIntegerTopic("camera_resolution_height").publish().set(cameraConfig.getHeight());
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(cameraConfig.getAutoExposure());
    configTable.getDoubleTopic("camera_exposure").publish().set(cameraConfig.getExposure());
    configTable.getIntegerTopic("camera_saturation").publish().set(cameraConfig.getSaturation());
    configTable.getIntegerTopic("camera_hue").publish().set(cameraConfig.getHue());
    configTable.getIntegerTopic("camera_auto_white_balance").publish().set(cameraConfig.getAutoWhiteBalance());
    configTable.getIntegerTopic("camera_white_balance").publish().set(cameraConfig.getWhiteBalance()); 
    configTable.getDoubleTopic("camera_gain").publish().set(cameraConfig.getGain());
    configTable.getDoubleTopic("camera_denoise").publish().set(cameraConfig.getDenoise());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(VisionConstants.aprilTagWidth);
    configTable.getIntegerArrayTopic("obj_lower_hsv").publish().set(VisionConstants.objLowerHSV);
    configTable.getIntegerArrayTopic("obj_upper_hsv").publish().set(VisionConstants.objUpperHSV);
    configTable.getIntegerTopic("obj_blender_ai_id").publish().set(VisionConstants.AITargets.BLUE_BOT.ordinal());

    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
    isRecordingPublisher.set(false);
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
    fieldCameraPosePublisher = configTable.getFloatArrayTopic("field_camera_pose").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();
    this.camIndex = i;
    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01667));
    objDetectObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01667));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
    fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

    slowPeriodicTimer.start();
  }

  @Override
  public void updateInputs(
      VisionIOInputs inputs) {
    boolean slowPeriodic = slowPeriodicTimer.advanceIfElapsed(1.0);

    // Update NT connection status
    inputs.ntConnected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remote_id.startsWith(this.deviceId)) {
        inputs.ntConnected = true;
        break;
      }
    }
    inputs.connected = inputs.ntConnected;
    inputs.name = deviceId;

    // Publish timestamp
    if (slowPeriodic) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
      eventNamePublisher.set(DriverStation.getEventName());
      matchTypePublisher.set(DriverStation.getMatchType().ordinal());
      matchNumberPublisher.set(DriverStation.getMatchNumber());
    }

    // Publish tag layout
    var aprilTagType = aprilTagLayoutSupplier.get();
    if (aprilTagType != lastAprilTagLayout) {
      lastAprilTagLayout = aprilTagType;
      tagLayoutPublisher.set(aprilTagType.getLayoutString());
    }
    Pose3d camPose = new Pose3d(RobotContainer.drivetrainS.getPose())
			.plus(GeomUtil.poseToTransform(VisionConstants.cameras[camIndex].getPose().get()));
    Quaternion quar = camPose.getRotation().getQuaternion();
    float[] nums = {
      (float) camPose.getX(), 
      (float) camPose.getY(),
      (float) camPose.getZ(),
      //quaternion
      (float) quar.getW(),
      (float) quar.getX(),
      (float) quar.getY(),
      (float) quar.getZ()
    };
    fieldCameraPosePublisher.accept(nums);
    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    inputs.timestamps_april = new double[aprilTagQueue.length];
    inputs.frames_april = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      inputs.timestamps_april[i] = aprilTagQueue[i].timestamp / 1000000.0;
      inputs.frames_april[i] = aprilTagQueue[i].value;
    }
    if (slowPeriodic) {
      inputs.fps_april = fpsAprilTagsSubscriber.get();
    }

    // Get object detection data
    var objDetectQueue = objDetectObservationSubscriber.readQueue();
    inputs.timestamps_obj = new double[objDetectQueue.length];
    inputs.frames_obj  = new double[objDetectQueue.length][];
    for (int i = 0; i < objDetectQueue.length; i++) {
      inputs.timestamps_obj[i] = objDetectQueue[i].timestamp / 1000000.0;
      inputs.frames_obj[i] = objDetectQueue[i].value;
    }
    if (slowPeriodic) {
      inputs.fps_obj  = fpsObjDetectSubscriber.get();
    }
  }

  @Override
  public void setRecording(boolean active) {
    isRecordingPublisher.set(active);
  }
}