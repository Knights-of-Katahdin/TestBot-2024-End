package frc.robot.commands.SubsystemCmds;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

// ONLY WORKS IF CAMERA HAS GOOD VIEW OF TAG
// LOOKS FOR TAG WITH INPUT CAMERA WITHOUT CONSIDRATION OF ROBOT POSE ON FIELD

public class TurnToTagCmd extends Command {

        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);
        private final ProfiledPIDController omegaController = new ProfiledPIDController(1.75, 0, 0, OMEGA_CONSTRAINTS);

       
        private final SwerveSubsystem swerveSubsystem;

        private boolean shouldFlipForAlliance, isFinished;

        private int tagToChase;


        private final PhotonCamera photonCamera;
        private Transform3d photonCameraTransform3d;  
        private Pose3d robotPose; 
        private double thetaOffset;

        private PhotonTrackedTarget lastTarget;
        private Transform3d flipTransform = new Transform3d(0,0,0,new Rotation3d(0,0,Math.PI));
 

        public TurnToTagCmd(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera, Transform3d photonCameraTransform3d,
                                int tagToChase, double thetaOffset, boolean shouldFlipForAlliance) {

            this.swerveSubsystem = swerveSubsystem;
            this.photonCamera = photonCamera;
            this.photonCameraTransform3d = photonCameraTransform3d;
            this.tagToChase = tagToChase;
            this.thetaOffset = thetaOffset;
            this.shouldFlipForAlliance = shouldFlipForAlliance;



            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);
    

            addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {

            if(shouldFlipForAlliance){
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    thetaOffset = -thetaOffset;
                    if(tagToChase == 6)
                        tagToChase = 5;
                    else if(tagToChase == 7)
                        tagToChase = 4;
                    else if(tagToChase == 8)
                        tagToChase = 3;
                    else if(tagToChase == 9)
                        tagToChase = 2;
                    else if(tagToChase == 10)
                        tagToChase = 1;
                    else if(tagToChase == 14)
                        tagToChase = 13;
                    else if(tagToChase == 15)
                        tagToChase = 12;
                    else if(tagToChase == 16)
                        tagToChase = 11;

                } 
            }

            robotPose = new Pose3d(
                0, 0, 0, new Rotation3d(0, 0, 0));

            isFinished = false;
            lastTarget = null;

            PhotonPipelineResult photonRes = null;

            for(int i = 0; i < 10; i++){
                List<PhotonPipelineResult> photonPipelineResultList = photonCamera.getAllUnreadResults();
                if (!photonPipelineResultList.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                    photonRes = photonPipelineResultList.get(photonPipelineResultList.size() - 1);
                    if(photonRes.hasTargets()){
                        var targetOpt = photonRes.getTargets().stream()
                                .filter(t -> t.getFiducialId() == tagToChase)
                                .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                                .findFirst();
                        if (targetOpt.isPresent())
                            i = 10;
                    } 
                    if(i == 9){
                        System.out.println("Camera could not see April Tag");
                        isFinished = true;
                    }
                } 
            }


            
            if (photonRes.hasTargets()) {
                // Find the tag we want to chase
                var targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == tagToChase)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();
                    if (targetOpt.isPresent()) {
                        var target = targetOpt.get();
                        lastTarget = target;
                        // Trasnform the camera's pose to the target's pose
                        var cameraTargetTransform = target.getBestCameraToTarget();
                        
                    robotPose = new Pose3d(
                        0, 0, 0, new Rotation3d(0, 0, 0));
                    robotPose = robotPose.transformBy(cameraTargetTransform); 
                    robotPose = robotPose.transformBy(flipTransform); 
                    robotPose = robotPose.transformBy(photonCameraTransform3d);
                    



                    }
                } 
             
                omegaController.reset(robotPose.getRotation().getZ());

        }

        @Override
        public void execute() {



            List<PhotonPipelineResult> photonPipelineResultList = photonCamera.getAllUnreadResults();
            if (!photonPipelineResultList.isEmpty()) {
                    // Camera processed a new frame since last
                    // Get the last one in the list.
                var photonRes = photonPipelineResultList.get(photonPipelineResultList.size() - 1);
                if (photonRes.hasTargets()) {
                    // Find the tag we want to chase
                    var targetOpt = photonRes.getTargets().stream()
                        .filter(t -> t.getFiducialId() == tagToChase)
                        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                        .findFirst();
                        if (targetOpt.isPresent()) {
                            var target = targetOpt.get();
            
                            // Transform the camera's pose to the target's pose
                            Transform3d cameraTargetTransform = target.getBestCameraToTarget();
                            
                            robotPose = new Pose3d(
                                0, 0, 0, new Rotation3d(0, 0, 0));

                            robotPose = robotPose.transformBy(cameraTargetTransform);
                            robotPose = robotPose.transformBy(flipTransform); 
                            robotPose = robotPose.transformBy(photonCameraTransform3d);


                            double range = Math.hypot(robotPose.getX(), robotPose.getY());
                            double rotationGoal = Math.atan2(-robotPose.getY(),robotPose.getX());
                            Pose2d goalPose2d = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(rotationGoal));    
                            Transform2d rotTransform = new Transform2d(0, 0, new Rotation2d(thetaOffset));
                            goalPose2d = goalPose2d.transformBy(rotTransform);            
                            

        
                        
                        
                            System.out.println("Robot X " +robotPose.getX());
                            System.out.println("Robot Y " +robotPose.getY());
                            System.out.println("Robot Theta " + Math.toDegrees(robotPose.getRotation().getZ()));
                            System.out.println("Rotation Goal " + Math.toDegrees(goalPose2d.getRotation().getRadians()));
                            System.out.println("Range " + range);
                            omegaController.setGoal(goalPose2d.getRotation().getRadians());

                        }
       
                    }
                } 
                if (lastTarget == null) {
                // No target has been visible
                swerveSubsystem.stopModules();
                System.out.println("null");
                } else {
                    // Turn to the target
            
                    var turningSpeed = omegaController.calculate(robotPose.getRotation().getZ());
                    turningSpeed = turningSpeed / swerveSubsystem.getThrottle();


                    if (omegaController.atGoal()) {
                        turningSpeed = 0;
                    }

                    swerveSubsystem.setChassisSpeeds(
                        new ChassisSpeeds(0,0, turningSpeed));
                }
                



            
            if(omegaController.atGoal())
                isFinished = true;

            
        }



        @Override
        public void end(boolean interrupted) {
            swerveSubsystem.stopModules();
        }


        @Override
        public boolean isFinished() {
            return this.isFinished;
        }
        

}