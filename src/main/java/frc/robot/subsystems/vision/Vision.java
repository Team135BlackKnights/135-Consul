package frc.robot.subsystems.vision;

import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.PVCameras;

public class Vision extends SubsystemChecker {
	private final VisionIO io;
	private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	private boolean override = false;
	private Timer timer = new Timer();

	public Vision(VisionIO io) { this.io = io; }

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("VisionS", inputs);
		if (timer.get() > 1) {
			timer.stop();
			override = false;
		}
		//Update the global pose for each camera
		for (int i = 0; i < 4; i++) {
			if (!inputs.estPose[i].equals(new Pose2d())) { //if not default
				int[] aprilTagList = {};
				PVCameras camera = PVCameras.getCameraByIndex(i);
				switch (camera) {
					case Front_Camera:
					aprilTagList= inputs.frontCamTagList;
						break;
					case Left_Camera:
					aprilTagList = inputs.leftCamTagList;
						break;
					case Right_Camera:
					aprilTagList = inputs.rightCamTagList;
						break;
					case Back_Camera:
					aprilTagList = inputs.backCamTagList;
						break;
					}
				String response = shouldAcceptVision(inputs.time[i],
						inputs.estPose[i], RobotContainer.drivetrainS.getPose(),
						RobotContainer.drivetrainS.getChassisSpeeds(),
						aprilTagList, inputs.avgPoseAmbiguity[i]);
				if (response == "OK") {
					// Change our trust in the measurement based on the tags we can see
					// We do this because we should trust cam estimates with closer apriltags than farther ones.
					Matrix<N3, N1> estStdDevs = getEstimationStdDevs(
							inputs.avgDist[i],inputs.lowestDist[i],inputs.weightAverage[i],inputs.avgPoseAmbiguity[i], aprilTagList.length);
					addVisionMeasurement(inputs.estPose[i], inputs.time[i],
							estStdDevs);
					for (int tag : aprilTagList) {
						VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
								.min(1,
										VisionConstants.FieldConstants.aprilTagOffsets[tag]
												+ .001);
					}
				} else {
					if (response == "Max correction") {
						if (inputs.avgDist[i] < VisionConstants.offsetMaxDistance){
							for (int tag : aprilTagList) {
								VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
										.max(0,
												VisionConstants.FieldConstants.aprilTagOffsets[tag]
														- .001);
							}
						}

					}
					if (response == "Min trust") {
						for (int tag : aprilTagList) {
							VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
									.min(1,
											VisionConstants.FieldConstants.aprilTagOffsets[tag]
													+ .001);
						}
					}
				}
				if (VisionConstants.debug) {
					SmartDashboard.putNumberArray("OFFSETS",
							VisionConstants.FieldConstants.aprilTagOffsets);
				}
			}
		}
	}

	/**
	 * Adds the vision measurement to the poseEstimator. This is the same as the
	 * default poseEstimator function, we just do it this way to fit our block
	 * template structure
	 */
	public void addVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		RobotContainer.drivetrainS.newVisionMeasurement(pose, timestamp,
				estStdDevs);
	}

	public String shouldAcceptVision(double timestamp, Pose2d newEst,
			Pose2d lastPosition, ChassisSpeeds robotVelocity, int[] aprilTagList,
			double averagePoseAmbig) {
		// Check out of field
		if (newEst.getTranslation()
				.getX() < -VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getX() > VisionConstants.FieldConstants.kFieldLength
								+ VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getY() < -VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getY() > VisionConstants.FieldConstants.kFieldWidth
								+ VisionConstants.FieldConstants.kFieldBorderMargin) {
			SmartDashboard.putString("Vision validation", "Outside field");
			return "Outside field";
		}
		double overallChange;
		boolean[] moduleSkids = RobotContainer.drivetrainS.isSkidding();
		if (moduleSkids[0] || moduleSkids[1] || moduleSkids[2] || moduleSkids[3]
				|| RobotContainer.drivetrainS.isCollisionDetected() || override) {
			if (RobotContainer.drivetrainS.isCollisionDetected()) {
				override = true;
				timer.restart();
			}
			if (GeomUtil.distancePose(lastPosition,
					newEst) > VisionConstants.kMaxVisionCorrectionSkid) {
				SmartDashboard.putString("Vision validation", "Max correction");
				return "Max correction";
			}
			if (Math
					.abs(newEst.getRotation().minus(lastPosition.getRotation())
							.getRadians()) > VisionConstants.kMaxRotationCorrectionSkid
					&& (RobotContainer.drivetrainS.isConnected())) {
				SmartDashboard.putString("Vision validation", "Max rotation");
				return "Max rotation";
			}
			if (averagePoseAmbig > .5) {
				SmartDashboard.putString("Vision validation", "Max ambiguity");
				return "Max ambiguity";
			}
			SmartDashboard.putString("Vision validation", "Override");
			return "OK";
		}
		//The following check for velocity MAY be unneeded!
		if (robotVelocity.vyMetersPerSecond == 0) {
			overallChange = Math.abs(robotVelocity.vxMetersPerSecond);
		} else {
			overallChange = Math.hypot(robotVelocity.vxMetersPerSecond,
					robotVelocity.vyMetersPerSecond);
		}
		if (overallChange > VisionConstants.kMaxVelocity) {
			SmartDashboard.putString("Vision validation", "Max velocity");
			return "Max velocity";
		}
		// Check max correction
		if (GeomUtil.distancePose(lastPosition,
				newEst) > VisionConstants.kMaxVisionCorrection) {
			SmartDashboard.putString("Vision validation", "Max correction");
			return "Max correction";
		}
		if (Math
				.abs(newEst.getRotation().minus(lastPosition.getRotation())
						.getRadians()) > VisionConstants.kMaxRotationCorrection
				&& (RobotContainer.drivetrainS.isConnected())) {
			SmartDashboard.putString("Vision validation", "Max rotation");
			return "Max rotation";
		}
		for (int aprilTagID : aprilTagList) {
			if (VisionConstants.FieldConstants.aprilTagOffsets[aprilTagID] < VisionConstants.FieldConstants.kFieldTagMinTrust) {
				SmartDashboard.putString("Vision validation", "Min trust");
			}
		}
		if (averagePoseAmbig > .3) {
			SmartDashboard.putString("Vision validation", "Max ambiguity");
			return "Max ambiguity";
		}
		SmartDashboard.putString("Vision validation", "OK");
		return "OK";
	}

	/**
	 * Compute the standard deviation based on the parameters given.
	 * 
	 * @param estimatedPose   the estimated pose that is being returned
	 * @param photonEstimator the pose estimator to use
	 * @param cam             the camera to use
	 * @return A matrix of the standard deviations
	 */
	private Matrix<N3, N1> getEstimationStdDevs(double avgDist, double lowestDist, double weightAverage, double avgPoseAmbiguity,
			int numTags) {
		double xyStdDev = calculateXYStdDev(avgDist,
		lowestDist, weightAverage, avgPoseAmbiguity,
				numTags);
		double thetaStdDev = calculateThetaStdDev(avgDist,
		lowestDist, weightAverage, avgPoseAmbiguity,
				numTags);
		xyStdDev = applyAdditionalAdjustments(xyStdDev, numTags);
		thetaStdDev = applyAdditionalAdjustments(thetaStdDev, numTags);
		return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
	}

	/**
	 * Calculate xyStdDev based on distance, average distance, average pose
	 * ambiguity, weigh average, and number of tags.
	 */
	private double calculateXYStdDev(double avgDist, double lowestDist,
			double weighAverage, double avgPoseAmbiguity, int numTags) {
		double distWeight = Math.pow(lowestDist / 2.0, 2.0);
		double avgDistWeight = Math.pow(avgDist / 3.5, 2.0);
		double poseWeight = Math.pow(avgPoseAmbiguity / 0.2, 2.0);
		double weighAverageWeight = 1-weighAverage;
		return VisionConstants.std_dev_multiplier
				* (distWeight + poseWeight + weighAverageWeight + avgDistWeight)
				/ (numTags * 2);
	}

	/**
	 * Calculate thetaStdDev based on distance, average distance, average pose
	 * ambiguity, weigh average, and number of tags.
	 */
	private double calculateThetaStdDev(double avgDist, double lowestDist,
			double weighAverage, double avgPoseAmbiguity, int numTags) {
		double distWeight = Math.pow(lowestDist / 2.0, 2.0);
		double avgDistWeight = Math.pow(avgDist / 3.5, 2.0);
		double poseWeight = Math.pow(avgPoseAmbiguity / 0.2, 2.0);
		double weighAverageWeight = Math.pow(1-weighAverage, 2.0);
		return VisionConstants.std_dev_multiplier
				* (distWeight + poseWeight + weighAverageWeight + avgDistWeight)
				/ (numTags * 2);
	}

	/**
	 * Apply any additional adjustments or constraints to the calculated standard
	 * deviation.
	 */
	private double applyAdditionalAdjustments(double stdDev, double tagCount) {
		boolean[] moduleSkids = RobotContainer.drivetrainS.isSkidding();
		for (boolean moduleSkid : moduleSkids) {
			if (moduleSkid) {
				stdDev /= 2;
			}
		}
		if (RobotContainer.drivetrainS.isCollisionDetected() || override) {
			stdDev /= 4;
		}
		if (moduleSkids[0] || moduleSkids[1] || moduleSkids[2]
				|| moduleSkids[3]) {
			return Math.max(0.05, stdDev);
		}
		// Add more adjustments as needed
		return Math.max(0.02, stdDev); // Minimum threshold
	}

	/**
	 * Computes the distance in inches from the limelight network table entry
	 * 
	 * @param tY the tY measurement from the limelight network table entry (pitch
	 *              degrees)
	 * @return distance in inches
	 */
	public static double calculateDistanceFromtY(double tY) {
		// Calculate the angle using trigonometry
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = VisionConstants.limeLightAngleOffsetDegrees;
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = VisionConstants.limelightLensHeightoffFloorInches;
		// distance from the target to the floor
		double goalHeightInches = 0; //if multiple targets, make this an argument
		double angleToGoalDegrees = limelightMountAngleDegrees + tY;
		//calculate distance
		double distance = (goalHeightInches - limelightLensHeightInches)
				/ Math.tan(Units.degreesToRadians(angleToGoalDegrees));
		return Math.abs(distance); //incase somehow the angle became pos when the object is below the target, and vice versa.
	}

	/**
	 * Create a field relative pose3d from a given pose3d
	 * 
	 * @param robotRelativePose the pose3d of the object RELATIVE to the robot
	 * @param robotPose         the robot pose
	 * @return field relative pose3d of the object
	 */
	public static Pose3d fieldRelativePose3d(Pose3d robotRelativePose,
			Pose2d robotPose) {
		return new Pose3d(robotPose)
				.transformBy(new Transform3d(robotRelativePose.getTranslation(),
						robotRelativePose.getRotation()));
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	@Override
	protected Command systemCheckCommand() { return Commands.none(); }

	@Override
	public double getCurrent() { return 0; }
}
