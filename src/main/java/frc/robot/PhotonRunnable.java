package frc.robot;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import java.util.List;
import java.util.function.BiConsumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final BiConsumer<Pose2d, Double> poseConsumer;




  public PhotonRunnable(PhotonCamera photonCamera, Transform3d ROBOT_TO_CAMERA, BiConsumer<Pose2d, Double> poseConsumer) {

    this.poseConsumer = poseConsumer;
    this.photonCamera = photonCamera;
    
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    // Origin will always be blue
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAMERA);

  }

  @Override
  public void run() {

     while (!Thread.interrupted()) {
 
        // Get AprilTag data

            List<PhotonPipelineResult> photonPipelineResultList = photonCamera.getAllUnreadResults();

            if (!photonPipelineResultList.isEmpty()) {
                  // Camera processed a new frame since last
                  // Get the last one in the list.
                  PhotonPipelineResult photonResults = photonPipelineResultList.get(photonPipelineResultList.size() - 1);

                  if (photonResults.hasTargets()
                              && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                              photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                              var estimatedPose = estimatedRobotPose.estimatedPose;
                              // Make sure the measurement is on the field
                              if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
                                    && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
                              
                                    poseConsumer.accept(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
                              }
                              });
                  }

            }

    }

  }


}
