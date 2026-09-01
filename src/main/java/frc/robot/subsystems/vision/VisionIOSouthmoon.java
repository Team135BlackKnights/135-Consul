package frc.robot.subsystems.vision;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.networktables.*;
import org.wpilib.util.WPIUtilJNI;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.system.Timer;
import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.vision.VisionConstants;

/**
 * Southmoon implementation of VisionIO.
 *
 * <p>This class is the robot-side bridge to the Mac vision process. It publishes
 * configuration under /{deviceId}/config and subscribes to results under
 * /{deviceId}/output. Keep topic names and packet layouts synchronized with the
 * Python OutputPublisher.
 */
public class VisionIOSouthmoon implements VisionIO {
  private final Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier;
  private VisionConstants.AprilTagLayoutType lastAprilTagLayout = null;

  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber objDetectObservationSubscriber;
  private final DoubleArraySubscriber objDetectTxySubscriber;
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
   * @param id NetworkTables device ID used by the Mac process
   * @param i Camera index matching VisionConstants.cameras
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

    // Publish camera configuration for the Mac to consume. Students should expect
    // these to appear in NetworkTables at /<deviceId>/config/*.
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
    // The Blender/pose lookup pipeline needs to know which object class is the
    // field object of interest. This ordinal must match the trained model class.
    configTable.getIntegerTopic("obj_blender_ai_id").publish().set(VisionConstants.AITargets.FUEL.ordinal());

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
    // keepDuplicates/sendAll/pollStorage preserve bursts of frames so Vision can
    // process camera timestamps accurately instead of only seeing the newest NT
    // value each robot loop.
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.KEEP_DUPLICATES,
                PubSubOption.SEND_ALL,
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01667));
    objDetectObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_observations")
            .subscribe(
                new double[] {},
                PubSubOption.KEEP_DUPLICATES,
                PubSubOption.SEND_ALL,
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01667));
    objDetectTxySubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_txy")
            .subscribe(
                new double[] {},
                PubSubOption.KEEP_DUPLICATES,
                PubSubOption.SEND_ALL,
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

    // Update NT connection status by checking for a connected client whose
    // remote_id starts with this device ID. If this is false, the Mac process is
    // not connected to the same NT server as the robot.
    inputs.ntConnected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remoteId.startsWith(this.deviceId)) {
        inputs.ntConnected = true;
        break;
      }
    }
    inputs.connected = inputs.ntConnected;
    inputs.name = deviceId;

    // Slow-changing match metadata is used by the Mac for recording filenames and
    // synchronization. Do not publish this every 20 ms unless there is a reason.
    if (slowPeriodic) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
      eventNamePublisher.set(MatchState.getEventName());
      matchTypePublisher.set(MatchState.getMatchType().ordinal());
      matchNumberPublisher.set(MatchState.getMatchNumber());
    }

    // Publish the AprilTag layout JSON whenever the selected layout changes. This
    // keeps robot-side pose filtering and coprocessor solvePnP on the same field.
    var aprilTagType = aprilTagLayoutSupplier.get();
    if (aprilTagType != lastAprilTagLayout) {
      lastAprilTagLayout = aprilTagType;
      tagLayoutPublisher.set(aprilTagType.getLayoutString());
    }
    // The Mac needs the current field-to-camera pose for object pose solving. This
    // is drivetrain pose plus the measured robot-to-camera transform.
    Pose3d camPose = new Pose3d(RobotContainer.drivetrainS.getPose())
			.plus(GeomUtil.poseToTransform(VisionConstants.cameras[camIndex].getPose().get()));
    Quaternion quar = camPose.getRotation().getQuaternion();
    float[] nums = {
      (float) camPose.getX(),
      (float) camPose.getY(),
      (float) camPose.getZ(),
      // Quaternion is ordered w, x, y, z to match WPILib/Python parsing.
      (float) quar.getW(),
      (float) quar.getX(),
      (float) quar.getY(),
      (float) quar.getZ()
    };
    fieldCameraPosePublisher.accept(nums);

    // Get AprilTag data. Each queued NT value is a complete Southmoon packet; the
    // parser in Vision.java interprets the doubles.
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

    // Get object detection tx/ty-only data. Packet layout:
    // [count, classId, confidence, txDeg, tyDeg, distanceMeters, ...]
    // This is the preferred custom object interface because it is lightweight and
    // lets robot code choose/cluster targets instead of trusting one "best" box.
    var objDetectTxyQueue = objDetectTxySubscriber.readQueue();
    ArrayList<ObjDetectTxyObservation> txyObservations = new ArrayList<>();
    for (int i = 0; i < objDetectTxyQueue.length; i++) {
      double timestamp = objDetectTxyQueue[i].timestamp / 1000000.0;
      double[] values = objDetectTxyQueue[i].value;
      if (values.length == 0) {
        continue;
      }
      int count = (int) values[0];
      int expectedLen = 1 + count * 5;
      // Be defensive around partially published or version-mismatched packets:
      // parse only complete detections and ignore the rest of the frame.
      int safeLen = Math.min(values.length, expectedLen);
      for (int idx = 0; idx < count; idx++) {
        int base = 1 + idx * 5;
        if (base + 4 >= safeLen) {
          break;
        }
        int classId = (int) values[base];
        double confidence = values[base + 1];
        double txDeg = values[base + 2];
        double tyDeg = values[base + 3];
        double distanceMeters = values[base + 4];
        txyObservations.add(
            new ObjDetectTxyObservation(
                classId,
                confidence,
                Rotation2d.fromDegrees(txDeg),
                Rotation2d.fromDegrees(tyDeg),
                distanceMeters,
                timestamp));
      }
    }
    inputs.objDetectTxyObservations = txyObservations.toArray(new ObjDetectTxyObservation[0]);

    // Get object detection data (legacy "best" + pose). If tx/ty-only data is
    // present, drain this queue but do not populate frames_obj so downstream code
    // does not process both protocols for the same camera frame.
    var objDetectQueue = objDetectObservationSubscriber.readQueue();
    if (inputs.objDetectTxyObservations.length == 0) {
      inputs.timestamps_obj = new double[objDetectQueue.length];
      inputs.frames_obj  = new double[objDetectQueue.length][];
      for (int i = 0; i < objDetectQueue.length; i++) {
        inputs.timestamps_obj[i] = objDetectQueue[i].timestamp / 1000000.0;
        inputs.frames_obj[i] = objDetectQueue[i].value;
      }
    } else {
      inputs.timestamps_obj = new double[] {};
      inputs.frames_obj = new double[][] {};
    }
    if (slowPeriodic) {
      inputs.fps_obj  = fpsObjDetectSubscriber.get();
    }
  }

  @Override
  public void setRecording(boolean active) {
    // The Mac process owns video writing; the robot just publishes the desired
    // state. Vision enables this automatically during FMS-attached matches.
    isRecordingPublisher.set(active);
  }
}
