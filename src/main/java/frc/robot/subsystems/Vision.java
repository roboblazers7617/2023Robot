// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera("eyeball");
  PhotonPipelineResult result;
  PhotonTrackedTarget bestTag;
  AprilTagFieldLayout layout;
  PhotonPoseEstimator poseEstimator;


  ShuffleboardTab vision;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable datatable = inst.getTable("Vision");
  DoublePublisher angleToTarget;
  DoublePublisher distanceToTarget;
  
  public Vision() {
  try{
    layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    }
    catch(IOException e){
      layout = null;
    }
    poseEstimator =  new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_LAST_POSE, camera, VisionConstants.CAMERA_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
    if(result.hasTargets()){
    bestTag = result.getBestTarget();
    }
    angleToTarget.set(getBestTagYaw());
  }

public double getBestTagDistance() {
  if(bestTag != null)
  return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT, VisionConstants.TAG_HEIGHT[bestTag.getFiducialId()], VisionConstants.CAMERA_PITCH, Units.degreesToRadians(bestTag.getPitch()));
  else
    return 0;
}

public double getBestTagYaw() {
  if(bestTag != null)
    return bestTag.getYaw();
  else
    return 0;
}

public double getBestTagId(){
  if(bestTag != null)
    return bestTag.getFiducialId();
  else
    return -1;
}


}
