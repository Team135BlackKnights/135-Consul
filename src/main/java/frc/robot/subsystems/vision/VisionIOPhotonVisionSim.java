package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.vision.VisionConstants;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim; // Singleton vision sim

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final String cameraName;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name         The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(Supplier<VisionConstants.AprilTagLayoutType> aprilTagLayoutSupplier,
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(aprilTagLayoutSupplier, name, robotToCamera);
    this.cameraName = name;
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayoutSupplier.get().getLayout());
    }
    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setAvgLatencyMs(50);
    cameraProperties.setLatencyStdDevMs(5);
    cameraProperties.setFPS(60);
    cameraProperties.setExposureTimeMs(6);
    cameraProperties.setCalibError(.2, .045);
    cameraProperties.setCalibration(1600, 1304,
    MatBuilder.fill(Nat.N3(), Nat.N3(),
        966.85371839149309, 0.0, 792.59524206028857, 0.0,
            966.95923170939261, 703.31666886872517, 0.0, 0.0, 1.0),
    VecBuilder.fill(
         0.18157627229471812, -0.24984502994628116, 0.0, 0.0,
            0.11545743579195793,0,0,0));
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long timestamp = System.currentTimeMillis();
    visionSim.update(poseSupplier.get());
    Logger.recordOutput("Vision/" + cameraName + "SimMS", System.currentTimeMillis() - timestamp);
    super.updateInputs(inputs); // Act as real camera.

  }
}