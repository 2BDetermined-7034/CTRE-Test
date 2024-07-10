// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;


public class Vision extends SubsystemBase {

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonCamera camera;
  private Transform3d robotToCam = new Transform3d(0d, 0d, 0.5d, new Rotation3d());
  private PhotonPipelineResult latestResult;
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

  //SIM
  private final boolean updateSim = true;
  private final boolean simWireframe = true;
  // A vision system sim labelled as "main" in NetworkTables
  private VisionSystemSim visionSim = new VisionSystemSim("main");
  // A 0.5 x 0.25 meter rectangular target
  private SimCameraProperties cameraProp;
  private PhotonCameraSim cameraSim;


  /** Creates a new Vision. */
  public Vision(String cameraName) {
    camera = new PhotonCamera(cameraName);

    visionSim.addAprilTags(aprilTagFieldLayout);
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    cameraProp.setCalibError(0.25, 0.08);
    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    cameraSim.enableDrawWireframe(simWireframe);
    visionSim.addCamera(cameraSim, robotToCam);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latestResult = camera.getLatestResult();
  }

  public boolean hasTargets() {
    return latestResult.hasTargets();
  }

  public List<PhotonTrackedTarget> getTargetList() {
    return latestResult.getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return latestResult.getBestTarget();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d referencePose3d) {
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
    if(updateSim) {
      updateVisionSim(referencePose3d);
    }
    return estimatedRobotPose;
  }

  private void updateVisionSim(Pose3d estimatedPose) {
    visionSim.update(estimatedPose);
  }

}
