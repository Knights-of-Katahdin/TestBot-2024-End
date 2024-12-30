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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToTagCmd extends Command {

        private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_SPEED_MPS, DriveConstants.PHYSICAL_MAX_ACC_MPS2);
        private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_SPEED_MPS, DriveConstants.PHYSICAL_MAX_ACC_MPS2);
        private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS,DriveConstants.PHYSICAL_MAX_ANG_SPEED_RPS2);
    
        private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
        private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
        private final ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);

       
        private final SwerveSubsystem swerveSubsystem;

        private boolean isFinished;

        private double x, y;
        private int tagToChase;


        private final PhotonCamera photonCamera;
        private Transform3d photonCameraTransform3d;  
        private Pose3d robotPose; 
        private boolean shouldFlipForAlliance;

        private PhotonTrackedTarget lastTarget;
        private Transform3d flipTransform = new Transform3d(0,0,0,new Rotation3d(0,0,Math.PI));

        public DriveToTagCmd(SwerveSubsystem swerveSubsystem, PhotonCamera photonCamera, Transform3d photonCameraTransform3d,
                                int tagToChase, double x, double y, boolean shouldFlipForAlliance) {

            this.swerveSubsystem = swerveSubsystem;
            this.shouldFlipForAlliance = shouldFlipForAlliance;
            this.photonCamera = photonCamera;
            this.photonCameraTransform3d = photonCameraTransform3d;
            this.shouldFlipForAlliance = shouldFlipForAlliance;
            this.tagToChase = tagToChase;
            this.x = x;
            this.y = y;

            xController.setTolerance(0.1);
            yController.setTolerance(0.1);
            omegaController.setTolerance(Units.degreesToRadians(2));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);
    

            addRequirements(swerveSubsystem);
        }

        @Override
        public void initialize() {

            if(shouldFlipForAlliance){
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
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
                        // Transform the camera's pose to the target's pose
                        var cameraTargetTransform = target.getBestCameraToTarget();
                        
                    robotPose = new Pose3d(
                        0, 0, 0, new Rotation3d(0, 0, 0));
                    robotPose = robotPose.transformBy(cameraTargetTransform); 
                    robotPose = robotPose.transformBy(flipTransform); 
                    robotPose = robotPose.transformBy(photonCameraTransform3d);
                    



                    }
                } 
             
                omegaController.reset(robotPose.getRotation().getZ());
                xController.reset(robotPose.getX());
                yController.reset(robotPose.getY());

        }

        @Override
        public void execute() {


            List<PhotonPipelineResult> photonPipelineResultList = photonCamera.getAllUnreadResults();
            if (!photonPipelineResultList.isEmpty()) {
                // Camera processed a new frame since last
                // Get the last one in the list.
                PhotonPipelineResult photonRes = photonPipelineResultList.get(photonPipelineResultList.size() - 1);

                if (photonRes.hasTargets()) {
                    // Find the tag we want to chase
                    var targetOpt = photonRes.getTargets().stream()
                        .filter(t -> t.getFiducialId() == tagToChase)
                        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                        .findFirst();
                    if (targetOpt.isPresent()) {
                        var target = targetOpt.get();
        
                        // Trasnform the camera's pose to the target's pose
                        Transform3d cameraTargetTransform = target.getBestCameraToTarget();
                        
                        robotPose = new Pose3d(
                            0, 0, 0, new Rotation3d(0, 0, 0));

                        robotPose = robotPose.transformBy(cameraTargetTransform);
                        robotPose = robotPose.transformBy(flipTransform); 
                        robotPose = robotPose.transformBy(photonCameraTransform3d);


                        Pose2d goalPose2d = new Pose2d(x, y, new Rotation2d(0));    
            
                        
                    
                        System.out.println("Robot X " +robotPose.getX());
                        System.out.println("Robot Y " +robotPose.getY());
                        System.out.println("Robot Theta " + Math.toDegrees(robotPose.getRotation().getZ()));
                        System.out.println("Goal X " + goalPose2d.getX());
                        System.out.println("Goal Y " + goalPose2d.getY());
                        System.out.println("Goal Theta " + Math.toDegrees(goalPose2d.getRotation().getRadians()));
                        omegaController.setGoal( goalPose2d.getRotation().getRadians());
                        xController.setGoal(goalPose2d.getX());
                        yController.setGoal(goalPose2d.getY());


 
       
                    }
                } 
            }
            if (lastTarget == null) {
                // No target has been visible
                swerveSubsystem.stopModules();
                System.out.println("null");
            } else {
                // Turn to the target
        
                double turningSpeed = omegaController.calculate(robotPose.getRotation().getZ());
                double xSpeed =  -xController.calculate(robotPose.getX());
                double ySpeed =  yController.calculate(robotPose.getY());

                turningSpeed = turningSpeed / swerveSubsystem.getThrottle();
                xSpeed = xSpeed / swerveSubsystem.getThrottle();
                ySpeed = ySpeed / swerveSubsystem.getThrottle();


                // Drive to the target
                
                if (xController.atGoal()) {
                    xSpeed = 0;
                }

                if (yController.atGoal()) {
                    ySpeed = 0;
                }
                if (omegaController.atGoal()) {
                    turningSpeed = 0;
                }


                swerveSubsystem.setChassisSpeeds(
                    new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
                }
                

                if(omegaController.atGoal() && xController.atGoal() && yController.atGoal())
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