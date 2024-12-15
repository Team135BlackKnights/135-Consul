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
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.cameraName = name;
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(VisionConstants.kTagLayout);
    }
    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraProperties.setAvgLatencyMs(15);
    cameraProperties.setFPS(50);
    cameraProperties.setCalibError(.61, .1);
    cameraProperties.setCalibration(800, 600, MatBuilder.fill(Nat.N3(), Nat.N3(), 451.5581214725775,0.0,405.274839970422,0.0,453.4013764542542,339.3210175619262,0.0,0.0,1.0), VecBuilder.fill(0.04572077478465107,-0.07645251582223457,0.011983192982840202,-0.0011585844737787593,0.0031396106128620486,-5.013737433037841E-4,-0.0034827234187051357,-0.007343273311848231));
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long timestamp = System.currentTimeMillis();
    visionSim.update(poseSupplier.get());
    Logger.recordOutput("Vision/"+cameraName+"SimMS",  System.currentTimeMillis() - timestamp);
    super.updateInputs(inputs); // Act as real camera.

  }
}